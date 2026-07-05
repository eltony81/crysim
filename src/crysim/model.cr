module CrySim
  class ModelError < Exception
  end

  # Raised at build time when a cycle made only of direct-feedthrough
  # blocks is found. The standard fixes: insert a stateful block (e.g. a
  # fast first-order dynamic or an integrator) somewhere in the loop.
  class AlgebraicLoopError < ModelError
  end

  # Raised at run time the first time any block produces a NaN/Inf value —
  # named to the specific block/port/time that produced it (division by
  # zero, an unstable pole, an out-of-range input to a custom Fn, ...)
  # instead of the failure surfacing many steps later as a mysteriously
  # broken plot. Carries enough to highlight the culprit in the diagram:
  # `Model#render_error(err, path)`.
  class NonFiniteValueError < ModelError
    getter block_name : String
    getter port : Int32
    getter time : Float64
    getter value : Float64

    def initialize(@block_name : String, @port : Int32, @time : Float64, @value : Float64)
      super(
        "block :#{@block_name} produced #{@value} at t=#{@time.round(6)} (output port #{@port}) — " \
        "check for division by zero, an unstable pole, or an out-of-range input")
    end
  end

  class SolverConfig
    property duration : Float64 = 1.0
    property dt : Float64 = 0.01
    property method : Symbol = :rk4
  end

  # One logged signal: label + where to read it (block index, output port),
  # plus display metadata (role, human-readable caption). `label` is a
  # String (not Symbol) so it can hold a subsystem-prefixed name — see
  # Wire's doc comment for why.
  record LogEntry, label : String, block_idx : Int32, port : Int32,
    role : Symbol, display : String?

  # Immutable, validated block diagram. Built by ModelBuilder; executed by
  # Engine; rendered by Diagram::SvgRenderer.
  class Model
    getter name : String
    getter blocks : Array(Block)
    getter wires : Array(Wire)
    getter config : SolverConfig
    getter block_index : Hash(String, Int32)
    getter eval_order : Array(Int32)
    getter log_entries : Array(LogEntry)
    # wires that close a cycle in the block graph (used by the renderer to
    # route them below the diagram)
    getter feedback_wires : Set(Wire)

    def initialize(@name : String, @blocks : Array(Block), @wires : Array(Wire), @config : SolverConfig)
      @block_index = {} of String => Int32
      @blocks.each_with_index do |b, i|
        raise ModelError.new("duplicate block name :#{b.name}") if @block_index.has_key?(b.name)
        @block_index[b.name] = i
      end
      validate_wires!
      validate_input_ports!
      @eval_order = compute_eval_order
      @feedback_wires = find_feedback_wires
      @log_entries = collect_log_entries
    end

    def run : SimResult
      Engine.new(self).run
    end

    def to_svg : String
      Diagram::SvgRenderer.new(self).to_svg
    end

    def render(path : String)
      Diagram::SvgRenderer.new(self).render(path)
    end

    # Combined report: diagram (with per-wire sparklines of `result`'s
    # logged signals) followed by the same plot panels as
    # `result.plot`. Delegates to SimResult#report.
    def report(result : SimResult, path : String)
      result.report(self, path)
    end

    # Renders the diagram with the block that raised `err` (see
    # NonFiniteValueError) highlighted in red — turns "the run blew up
    # somewhere" into "here":
    #
    #   begin
    #     result = model.run
    #   rescue err : CrySim::NonFiniteValueError
    #     model.render_error(err, "diagnosis.html")
    #   end
    def render_error(err : NonFiniteValueError, path : String)
      Diagram::SvgRenderer.new(self, highlight: err.block_name).render(path)
    end

    # LTI flattening fast-path (v0.3, deliberately narrow): reduces the
    # model to a single CrySpace::StateSpace via cryspace's `*` (series)
    # operator, when the model is EXACTLY one source feeding a simple
    # chain of continuous, SISO, LTI blocks (ss/dss/tf) into exactly one
    # sink — no branching, no fan-in, no feedback, no non-LTI block
    # (Gain/Sum/PID/Probe/...) anywhere in the model. This is intentionally
    # the "safe common case," not a general subgraph reducer: anything
    # more elaborate (branches, feedback, mixed continuous/discrete)
    # raises with a clear message rather than guessing. See
    # PIANO_FEATURE.md for why the general case is deferred.
    def to_state_space : CrySpace::StateSpace
      chain = lti_chain_or_raise
      chain.map { |bi| @blocks[bi].state_space.not_nil! }.reduce { |acc, sys| acc * sys }
    end

    # Simulates via the flattened StateSpace (CrySpace's own vectorized
    # `simulate`) instead of the general co-simulation engine — only the
    # final chain output is available (no intermediate taps), logged under
    # whatever label/role/display the sink wire already carries.
    def run_fast : SimResult
      chain = lti_chain_or_raise
      sys = chain.map { |bi| @blocks[bi].state_space.not_nil! }.reduce { |acc, s| acc * s }

      n_steps = (@config.duration / @config.dt).round.to_i
      raise ModelError.new("duration too short for dt") if n_steps < 1
      t = Array(Float64).new(n_steps + 1) { |k| k * @config.dt }

      src_idx = @blocks.index { |b| b.category == :source }.not_nil!
      source = @blocks[src_idx].as(Blocks::Source)
      u = [t.map { |ti| source.value(ti) }].to_tensor

      _, _, y_matrix = sys.simulate(t.to_tensor, u: u, method: @config.method)
      y = Array(Float64).new(n_steps + 1) { |i| y_matrix[i, 0].value }

      sink_wire = @wires.find { |w| @block_index[w.src] == chain.last }.not_nil!
      label = wire_signal_label(sink_wire).not_nil!
      meta = {label => SignalMeta.new(wire_role(sink_wire), wire_display(sink_wire))}
      sink_block = @blocks[@block_index[sink_wire.dst]]
      title = sink_block.is_a?(Blocks::Scope) ? sink_block.title : sink_block.name
      SimResult.new(@name, t, {label => y}, [ScopeView.new(title, [label])], meta)
    end

    # Validates and returns the ordered LTI block indices for
    # to_state_space/run_fast, or raises a specific ModelError naming what
    # disqualified the model from the fast-path.
    private def lti_chain_or_raise : Array(Int32)
      sources = (0...@blocks.size).select { |i| @blocks[i].category == :source }
      raise ModelError.new("to_state_space: needs exactly one source block driving the chain (found #{sources.size})") unless sources.size == 1
      raise ModelError.new("to_state_space: needs at least one sink") if @blocks.none? { |b| b.category == :sink }
      raise ModelError.new("to_state_space: feedback wires are not supported by the fast-path") unless @feedback_wires.empty?

      middle = (0...@blocks.size).reject { |i| @blocks[i].category == :source || @blocks[i].category == :sink }
      raise ModelError.new("to_state_space: model has no LTI blocks to combine") if middle.empty?
      middle.each do |i|
        sys = @blocks[i].state_space
        unless sys
          raise ModelError.new(
            "to_state_space: block :#{@blocks[i].name} is not LTI (no state_space) — the fast-path only " \
            "supports a simple chain of ss/dss/tf blocks between one source and one sink")
        end
        raise ModelError.new("to_state_space: fast-path v0.3 supports continuous chains only (block :#{@blocks[i].name} is discrete)") if sys.dt
        unless sys.n_inputs == 1 && sys.n_outputs == 1
          raise ModelError.new("to_state_space: block :#{@blocks[i].name} is not SISO — the fast-path only supports single-input/single-output chains")
        end
      end

      order = [] of Int32
      visited = Set(Int32).new
      current = sources.first
      loop do
        outs = @wires.select { |w| @block_index[w.src] == current }
        raise ModelError.new("to_state_space: block :#{@blocks[current].name} must have exactly one outgoing wire for the fast-path (has #{outs.size})") unless outs.size == 1
        nxt = @block_index[outs.first.dst]
        break if @blocks[nxt].category == :sink
        raise ModelError.new("to_state_space: block :#{@blocks[nxt].name} forms a cycle — the fast-path doesn't support feedback") if visited.includes?(nxt)
        ins = @wires.select { |w| @block_index[w.dst] == nxt }
        raise ModelError.new("to_state_space: block :#{@blocks[nxt].name} must have exactly one incoming wire for the fast-path (has #{ins.size})") unless ins.size == 1
        visited << nxt
        order << nxt
        current = nxt
      end

      unless order.size == middle.size && order.to_set == middle.to_set
        raise ModelError.new("to_state_space: the model has blocks outside the simple source -> chain -> sink path")
      end
      order
    end

    # Extracts the CrySpace::StateSpace underlying a block of this model
    # (ss, dss, tf) for direct analysis without leaving the model, e.g.
    # `model.state_space_of(:plant).poles` or `.bode_plot(...)`. Raises for
    # unknown blocks or blocks with no LTI representation (Gain, PID, ...).
    # Accepts a String too, for a subsystem-internal block ("m1.dynamics").
    def state_space_of(name : Symbol) : CrySpace::StateSpace
      state_space_of(name.to_s)
    end

    def state_space_of(name : String) : CrySpace::StateSpace
      idx = @block_index[name]? || raise ModelError.new("state_space_of: unknown block :#{name}")
      @blocks[idx].state_space || raise ModelError.new(
        "state_space_of: block :#{name} (#{@blocks[idx].class}) has no state-space representation")
    end

    # Signal name shown for a wire: the explicit `as:` label, or (when the
    # wire feeds a sink) an automatic name derived from the source block,
    # or nil for internal, unlabeled wires that carry no logged signal.
    # Raises if the source has more than one real output and no `as:` was
    # given — auto-naming individual ports (e.g. "motor_0") isn't possible
    # to do dynamically (see Wire's doc comment), so an explicit label is
    # required to disambiguate instead.
    def wire_signal_label(w : Wire) : String?
      return w.label if w.label
      return nil unless @blocks[@block_index[w.dst]].category == :sink
      src = @blocks[@block_index[w.src]]
      if src.n_outputs > 1
        raise ModelError.new(
          "connect :#{w.src} (port #{w.src_port}) to a scope: source has #{src.n_outputs} outputs — " \
          "add an explicit `as:` label to name the specific signal being logged")
      end
      w.src
    end

    # Role of the signal carried by a wire: an explicit `role:` override,
    # or inferred from the source block's category — sources are inputs,
    # probes are monitors, everything else (dynamics, math) is an output.
    def wire_role(w : Wire) : Symbol
      return w.role.not_nil! if w.role
      case @blocks[@block_index[w.src]].category
      when :source then :input
      when :probe  then :monitor
      else              :output
      end
    end

    # Optional human-readable caption overriding the raw signal name in the
    # SVG diagram and in plots.
    def wire_display(w : Wire) : String?
      w.display
    end

    # (src_idx, src_port) feeding each input port, indexed [block][port].
    def input_sources : Array(Array(Tuple(Int32, Int32)))
      table = @blocks.map { |b| Array(Tuple(Int32, Int32)).new(b.n_inputs, {0, 0}) }
      @wires.each do |w|
        table[@block_index[w.dst]][w.dst_port] = {@block_index[w.src], w.src_port}
      end
      table
    end

    private def validate_wires!
      @wires.each do |w|
        [w.src, w.dst].each do |n|
          raise ModelError.new("wire references unknown block :#{n}") unless @block_index.has_key?(n)
        end
        src = @blocks[@block_index[w.src]]
        dst = @blocks[@block_index[w.dst]]
        unless w.src_port >= 0 && w.src_port < src.n_outputs
          raise ModelError.new("wire from :#{w.src} port #{w.src_port}: block has #{src.n_outputs} output(s)")
        end
        unless w.dst_port >= 0 && w.dst_port < dst.n_inputs
          raise ModelError.new("wire into :#{w.dst} port #{w.dst_port}: block has #{dst.n_inputs} input(s)")
        end
      end
    end

    private def validate_input_ports!
      counts = @blocks.map { |b| Array(Int32).new(b.n_inputs, 0) }
      @wires.each { |w| counts[@block_index[w.dst]][w.dst_port] += 1 }
      counts.each_with_index do |ports, bi|
        ports.each_with_index do |c, pi|
          name = @blocks[bi].name
          raise ModelError.new("input #{pi} of :#{name} is not connected") if c == 0
          raise ModelError.new("input #{pi} of :#{name} has #{c} incoming wires (exactly 1 allowed)") if c > 1
        end
      end
    end

    # Dependency graph for output evaluation: block v depends on block u
    # when a wire u->v exists AND v has direct feedthrough (its output
    # needs its input *now*). Stateful/sampled blocks break the chain,
    # exactly as in Simulink. A cycle here is an algebraic loop.
    private def compute_eval_order : Array(Int32)
      n = @blocks.size
      deps = Array.new(n) { [] of Int32 } # deps[v] = blocks v depends on
      adj = Array.new(n) { [] of Int32 }  # adj[u]  = blocks depending on u
      @wires.each do |w|
        u = @block_index[w.src]
        v = @block_index[w.dst]
        next unless @blocks[v].direct_feedthrough?
        deps[v] << u
        adj[u] << v
      end

      in_deg = deps.map(&.size)
      queue = Deque(Int32).new
      n.times { |i| queue << i if in_deg[i] == 0 }
      order = [] of Int32
      while node = queue.shift?
        order << node
        adj[node].each do |v|
          in_deg[v] -= 1
          queue << v if in_deg[v] == 0
        end
      end

      if order.size < n
        cycle = (0...n).select { |i| in_deg[i] > 0 }.map { |i| ":#{@blocks[i].name}" }
        raise AlgebraicLoopError.new(
          "algebraic loop detected among feedthrough blocks #{cycle.join(" -> ")}; " \
          "insert a stateful block (integrator, transfer function or unit delay) in the loop")
      end
      order
    end

    # DFS back-edge classification over the full wire graph, used only for
    # diagram routing (feedback wires drawn below the diagram).
    private def find_feedback_wires : Set(Wire)
      n = @blocks.size
      adj = Array.new(n) { [] of Wire }
      @wires.each { |w| adj[@block_index[w.src]] << w }
      color = Array(Int32).new(n, 0) # 0 white, 1 grey, 2 black
      feedback = Set(Wire).new

      visit = uninitialized Proc(Int32, Nil)
      visit = ->(u : Int32) do
        color[u] = 1
        adj[u].each do |w|
          v = @block_index[w.dst]
          if color[v] == 0
            visit.call(v)
          elsif color[v] == 1
            feedback << w
          end
        end
        color[u] = 2
        nil
      end

      n.times { |i| visit.call(i) if color[i] == 0 }
      feedback
    end

    # Signals recorded during the run: labeled wires, wires entering a
    # Scope (auto-named after their source), probe outputs, and named
    # states of ss/dss blocks (state_names:, role :state).
    private def collect_log_entries : Array(LogEntry)
      entries = [] of LogEntry
      seen = Set(String).new
      add = ->(label : String, bi : Int32, port : Int32, role : Symbol, display : String?) do
        entries << LogEntry.new(label, bi, port, role, display) unless seen.includes?(label)
        seen << label
        nil
      end

      @wires.each do |w|
        next unless label = wire_signal_label(w)
        add.call(label, @block_index[w.src], w.src_port, wire_role(w), wire_display(w))
      end
      @blocks.each_with_index do |b, i|
        add.call(b.name, i, 0, :monitor, nil) if b.auto_log?
        if ports = b.state_ports
          ports.each { |name, port| add.call(name.to_s, i, port, :state, nil) }
        end
      end
      entries
    end
  end
end
