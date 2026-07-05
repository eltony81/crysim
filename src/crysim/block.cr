module CrySim
  # A directed connection between two block ports.
  #
  # `src`/`dst` are block *identities*, not the bare symbols the user types
  # in the DSL: a plain `connect :a, to: :b` identity is just `"a"`/`"b"`,
  # but a block declared inside a subsystem instance is identified by its
  # prefixed String (`"m1.dynamics"`) — Crystal symbols can't be
  # constructed dynamically (there is no `String#to_sym`), so this
  # composite identity has to be a String; see ModelBuilder#prefixed.
  #
  # `label` names the signal carried by the wire; labeled signals are logged
  # automatically into SimResult. `role` classifies the signal for display
  # (:input, :output, :monitor, ...) — always a literal symbol the user or
  # the engine chose, never synthesized, so it stays a Symbol. `display` is
  # an optional human-readable caption shown in the SVG diagram and in
  # plots instead of the raw label.
  record Wire,
    src : String,
    src_port : Int32,
    dst : String,
    dst_port : Int32,
    label : String? = nil,
    role : Symbol? = nil,
    display : String? = nil

  # Abstract base for every diagram block (the S-function equivalent).
  #
  # A block is defined by its output equation and, when stateful, its state
  # equation:
  #
  #   y  = output(t, x, u)
  #   x' = derivative(t, x, u)      (continuous state)
  #
  # Blocks whose output depends instantaneously on their input must report
  # `direct_feedthrough? == true`: the engine uses this to compute the
  # evaluation order and to detect algebraic loops.
  #
  # Sampled blocks (PID, Noise) update once per macro step via
  # `update_sample` and hold their output constant during solver substeps.
  abstract class Block
    # Final identity used for wiring/lookup — see Wire's doc comment.
    # Plain (unprefixed) outside a subsystem body.
    getter name : String
    property n_inputs : Int32
    getter n_outputs : Int32

    # Set by ModelBuilder#add_block; backs the `>>` chain sugar below.
    property builder : ModelBuilder? = nil

    def initialize(@name : String, @n_inputs : Int32, @n_outputs : Int32)
    end

    # Chain sugar: `step(:r) >> sum(:e) >> pid(:c)` wires port 0 of each
    # block into port 0 of the next (equivalent to `connect self, to:
    # other`) and returns `other`, so the chain keeps flowing. Only covers
    # the straight-line case — fan-out, feedback, and non-zero ports still
    # need an explicit `connect`/`feedback`. Uses each block's already-
    # final `.name` directly (bypassing prefixing/alias resolution, which
    # only apply to bare symbols the user types in `connect`).
    def >>(other : Block) : Block
      b = builder || raise ModelError.new("block :#{name} has no owning builder for >>")
      b.wire_connect(name, 0, other.name, 0)
      other
    end

    # Does the output depend instantaneously on the input?
    def direct_feedthrough? : Bool
      n_inputs > 0
    end

    # Number of continuous states.
    def state_size : Int32
      0
    end

    def initial_state : Array(Float64)
      Array(Float64).new
    end

    # Sinks accept any number of incoming wires: each new connection
    # appends an input port.
    def elastic_inputs? : Bool
      false
    end

    # Updated once per macro step (held during solver substeps)?
    def sampled? : Bool
      false
    end

    # How many base solver steps between updates of a sampled block (>=1;
    # default: every step, i.e. the base rate). A dss/DiscreteStateSpaceBlock
    # computes this from its own declared dt during `prepare`; PID accepts
    # an explicit `rate:` multiplier. The engine only calls `update_sample`
    # every `rate_steps`-th step, passing the actual elapsed time
    # (`rate_steps * base_dt`) so a slower-rate block's own math sees the
    # correct interval — this is v0.3's "simple multi-rate" (integer
    # multiples of the base dt only).
    def rate_steps : Int32
      1
    end

    # Called once before the run starts (dt is the solver step).
    def prepare(dt : Float64)
    end

    # Output equation: writes the block outputs into `y` (size n_outputs).
    abstract def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))

    # State equation: writes x' into `dx` (size state_size).
    def derivative(t : Float64, x : Array(Float64), u : Array(Float64), dx : Array(Float64))
    end

    # Per-macro-step update for sampled blocks. Called once per step, right
    # after the first output evaluation; a second output evaluation follows
    # immediately so downstream blocks and logs see the fresh result within
    # the same step (e.g. a PID's output must reach the plant it drives in
    # the step it was computed). A block that holds a genuine one-step
    # state (rather than directly producing its held output, like
    # PID/Noise) must NOT mutate the state read by `output` here — stage
    # the next state instead and promote it in `commit_sample`, otherwise
    # `output` would leak next-step's state into the current step (and into
    # every RK4 substep evaluation within it).
    def update_sample(t : Float64, u : Array(Float64), dt : Float64)
    end

    # Called once per macro step, after the continuous state has been
    # advanced, to promote any state staged in `update_sample` so it
    # becomes visible starting the next step. No-op for sampled blocks
    # that don't hold a two-buffer state (PID, Noise).
    def commit_sample(dt : Float64)
    end

    # Should the engine log this block's output automatically (probes)?
    def auto_log? : Bool
      false
    end

    # The CrySpace::StateSpace underlying this block, when it has one
    # (StateSpaceBlock, DiscreteStateSpaceBlock, and TransferFcn which
    # extends it) — nil for blocks with no LTI representation (Gain, PID,
    # sources, ...). Backs Model#state_space_of.
    def state_space : CrySpace::StateSpace?
      nil
    end

    # Named internal states exposed as extra output ports for automatic
    # per-state logging and readable diagram tooltips (see ss/dss
    # `state_names:`). Maps state name to its output port index; nil when
    # the block declares none. Backs Model#collect_log_entries.
    def state_ports : Hash(Symbol, Int32)?
      nil
    end

    # --- metadata used by the SVG renderer ---

    def category : Symbol
      :generic
    end

    def glyph_label : String
      self.class.name.split("::").last
    end

    # Parameters shown in the diagram tooltip.
    def params_description : String
      ""
    end
  end
end
