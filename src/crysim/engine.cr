module CrySim
  # Fixed-step co-simulation engine (strategy A of the design analysis).
  #
  # Per macro step:
  #   1. evaluate all block outputs in topological order at (t_k, x_k)
  #   2. update sampled blocks (PID, Noise) and re-evaluate so downstream
  #      blocks and logs see the fresh sampled outputs
  #   3. log signals
  #   4. advance the concatenated continuous state with Euler or RK4;
  #      sampled outputs are held constant during substeps (ZOH)
  class Engine
    @n : Int32
    @in_srcs : Array(Array(Tuple(Int32, Int32)))
    @sizes : Array(Int32)
    @offsets : Array(Int32)
    @n_states : Int32
    @outputs : Array(Array(Float64))
    @inputs : Array(Array(Float64))
    @x_locals : Array(Array(Float64))
    @dx_locals : Array(Array(Float64))
    @sampled : Array(Int32)
    @stateful : Array(Int32)

    def initialize(@model : Model)
      blocks = @model.blocks
      @n = blocks.size
      @in_srcs = @model.input_sources

      # continuous state layout: block i owns x[@offsets[i], @sizes[i]]
      @sizes = blocks.map(&.state_size)
      @offsets = [] of Int32
      total = 0
      @sizes.each do |s|
        @offsets << total
        total += s
      end
      @n_states = total

      @outputs = blocks.map { |b| Array(Float64).new(b.n_outputs, 0.0) }
      @inputs = blocks.map { |b| Array(Float64).new(b.n_inputs, 0.0) }
      @x_locals = blocks.map { |b| Array(Float64).new(b.state_size, 0.0) }
      @dx_locals = blocks.map { |b| Array(Float64).new(b.state_size, 0.0) }

      @sampled = [] of Int32
      blocks.each_with_index { |b, i| @sampled << i if b.sampled? }
      @stateful = [] of Int32
      blocks.each_with_index { |b, i| @stateful << i if b.state_size > 0 }
    end

    def run : SimResult
      cfg = @model.config
      dt = cfg.dt
      raise ModelError.new("dt must be positive") unless dt > 0.0
      n_steps = (cfg.duration / dt).round.to_i
      raise ModelError.new("duration too short for dt") if n_steps < 1

      blocks = @model.blocks
      blocks.each(&.prepare(dt))

      x = Array(Float64).new(@n_states, 0.0)
      @stateful.each do |bi|
        blocks[bi].initial_state.each_with_index { |v, j| x[@offsets[bi] + j] = v }
      end

      k1 = Array(Float64).new(@n_states, 0.0)
      k2 = Array(Float64).new(@n_states, 0.0)
      k3 = Array(Float64).new(@n_states, 0.0)
      k4 = Array(Float64).new(@n_states, 0.0)
      xtmp = Array(Float64).new(@n_states, 0.0)

      entries = @model.log_entries
      t_log = Array(Float64).new(n_steps + 1)
      logs = entries.map { Array(Float64).new(n_steps + 1) }

      (0..n_steps).each do |k|
        t = k * dt

        eval_outputs(t, x)
        due_any = false
        @sampled.each do |bi|
          next unless k % blocks[bi].rate_steps == 0
          due_any = true
          gather_inputs(bi)
          blocks[bi].update_sample(t, @inputs[bi], dt * blocks[bi].rate_steps)
        end
        eval_outputs(t, x) if due_any

        t_log << t
        entries.each_with_index do |e, i|
          logs[i] << @outputs[e.block_idx][e.port]
        end

        break if k == n_steps

        case cfg.method
        when :euler
          derivatives(t, x, k1)
          @n_states.times { |i| x[i] += dt * k1[i] }
        when :midpoint
          # 2-stage explicit RK2: only samples at t and t+dt/2, never at
          # the interval's right edge — unlike RK4 (see :rk4 below), this
          # doesn't miss an impulse pulse that's exactly one step wide,
          # and converges to the analytic impulse response as dt -> 0
          # instead of a fixed, dt-independent bias. Also 2nd-order
          # accurate (better than :euler) for everything else in a model.
          derivatives(t, x, k1)
          @n_states.times { |i| xtmp[i] = x[i] + 0.5 * dt * k1[i] }
          derivatives(t + 0.5 * dt, xtmp, k2)
          @n_states.times { |i| x[i] += dt * k2[i] }
        else # :rk4
          derivatives(t, x, k1)
          @n_states.times { |i| xtmp[i] = x[i] + 0.5 * dt * k1[i] }
          derivatives(t + 0.5 * dt, xtmp, k2)
          @n_states.times { |i| xtmp[i] = x[i] + 0.5 * dt * k2[i] }
          derivatives(t + 0.5 * dt, xtmp, k3)
          @n_states.times { |i| xtmp[i] = x[i] + dt * k3[i] }
          derivatives(t + dt, xtmp, k4)
          @n_states.times { |i| x[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) }
        end

        @sampled.each { |bi| blocks[bi].commit_sample(dt) }
      end

      signals = {} of String => Array(Float64)
      meta = {} of String => SignalMeta
      entries.each_with_index do |e, i|
        signals[e.label] = logs[i]
        meta[e.label] = SignalMeta.new(e.role, e.display)
      end
      scopes = @model.blocks.select(&.category.== :sink).map do |b|
        keys = @model.wires.select { |w| w.dst == b.name }.compact_map { |w| @model.wire_signal_label(w) }
        title = b.is_a?(Blocks::Scope) ? b.title : b.name.to_s
        ScopeView.new(title, keys)
      end
      SimResult.new(@model.name, t_log, signals, scopes, meta)
    end

    private def gather_inputs(bi : Int32)
      srcs = @in_srcs[bi]
      buf = @inputs[bi]
      srcs.each_with_index do |(si, sp), port|
        buf[port] = @outputs[si][sp]
      end
    end

    private def load_state(bi : Int32, x : Array(Float64))
      off = @offsets[bi]
      loc = @x_locals[bi]
      @sizes[bi].times { |j| loc[j] = x[off + j] }
    end

    private def eval_outputs(t : Float64, x : Array(Float64))
      blocks = @model.blocks
      @model.eval_order.each do |bi|
        gather_inputs(bi)
        load_state(bi, x)
        blocks[bi].output(t, @x_locals[bi], @inputs[bi], @outputs[bi])
        check_finite!(bi, t)
      end
    end

    # Fails fast, right at the block that produced it, instead of letting
    # a NaN/Inf silently propagate for the rest of the run and surface
    # many steps later as a mysteriously broken plot.
    private def check_finite!(bi : Int32, t : Float64)
      @outputs[bi].each_with_index do |v, port|
        next if v.finite?
        raise NonFiniteValueError.new(@model.blocks[bi].name, port, t, v)
      end
    end

    private def derivatives(t : Float64, x : Array(Float64), dx : Array(Float64))
      eval_outputs(t, x)
      blocks = @model.blocks
      @stateful.each do |bi|
        gather_inputs(bi)
        load_state(bi, x)
        blocks[bi].derivative(t, @x_locals[bi], @inputs[bi], @dx_locals[bi])
        off = @offsets[bi]
        @sizes[bi].times { |j| dx[off + j] = @dx_locals[bi][j] }
      end
    end
  end
end
