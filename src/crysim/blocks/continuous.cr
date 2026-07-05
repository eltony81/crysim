module CrySim
  module Blocks
    class Integrator < Block
      def initialize(name : String, x0 : Number = 0.0)
        super(name, 1, 1)
        @x0 = x0.to_f64
      end

      def direct_feedthrough? : Bool
        false
      end

      def state_size : Int32
        1
      end

      def initial_state : Array(Float64)
        [@x0]
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        y[0] = x[0]
      end

      def derivative(t : Float64, x : Array(Float64), u : Array(Float64), dx : Array(Float64))
        dx[0] = u[0]
      end

      def category : Symbol
        :continuous
      end

      def glyph_label : String
        "1/s"
      end

      def params_description : String
        "x0: #{@x0}"
      end
    end

    # Discrete unit delay (z⁻¹): y[k] = u[k-1], always at the base solver
    # rate. The direct discrete equivalent of Integrator — the single most
    # primitive block for hand-built discrete dynamics. Uses the same
    # two-buffer commit pattern as DiscreteStateSpaceBlock (held input,
    # promoted once per step after everything else has read it), but needs
    # no CrySpace::StateSpace at all since y[k]=u[k-1] has no real matrices.
    class UnitDelay < Block
      def initialize(name : String, x0 : Number = 0.0)
        super(name, 1, 1)
        @x0 = x0.to_f64
        @held = @x0
        @next_held = @x0
      end

      def direct_feedthrough? : Bool
        false
      end

      def sampled? : Bool
        true
      end

      def prepare(dt : Float64)
        @held = @x0
        @next_held = @x0
      end

      def update_sample(t : Float64, u : Array(Float64), dt : Float64)
        @next_held = u[0]
      end

      def commit_sample(dt : Float64)
        @held = @next_held
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        y[0] = @held
      end

      def category : Symbol
        :continuous
      end

      def glyph_label : String
        "z⁻¹"
      end

      def params_description : String
        "x0: #{@x0}"
      end
    end

    # Continuous LTI state-space block. Matrices are taken from a
    # CrySpace::StateSpace (built directly or converted from a transfer
    # function) and unpacked into plain arrays for the per-step hot path.
    class StateSpaceBlock < Block
      getter n_states : Int32
      getter sys : CrySpace::StateSpace
      @a : Array(Array(Float64))
      @b : Array(Array(Float64))
      @c : Array(Array(Float64))
      @d : Array(Array(Float64))
      @x0 : Array(Float64)
      @feedthrough : Bool
      @label : String
      @n_real_outputs : Int32
      @state_names : Array(Symbol)?
      @output_names : Array(Symbol)?

      def initialize(name : String, ss : CrySpace::StateSpace, x0 : Array(Float64)? = nil, label : String? = nil,
                     state_names : Array(Symbol)? = nil, output_names : Array(Symbol)? = nil)
        if dt = ss.dt
          raise ModelError.new(
            "blocco :#{name}: sistema discreto (dt=#{dt}) passato a un blocco continuo; " \
            "usa dss per un sistema discreto esplicito, oppure ss id, sys: ... che seleziona " \
            "automaticamente il blocco giusto in base al dt del sistema")
        end
        n = ss.a.shape[0]
        m = ss.b.shape.size > 1 ? ss.b.shape[1] : 1
        p = ss.c.shape[0]
        if state_names && state_names.size != n
          raise ModelError.new("blocco :#{name}: state_names deve avere #{n} elementi (ricevuti #{state_names.size})")
        end
        if output_names && output_names.size != p
          raise ModelError.new("blocco :#{name}: output_names deve avere #{p} elementi (ricevuti #{output_names.size})")
        end
        super(name, m, p + (state_names.try(&.size) || 0))
        @sys = ss
        @n_states = n
        @n_real_outputs = p
        @state_names = state_names
        @output_names = output_names
        @a = unpack(ss.a, n, n)
        @b = unpack(ss.b, n, m)
        @c = unpack(ss.c, p, n)
        @d = unpack(ss.d, p, m)
        @x0 = x0 || Array(Float64).new(n, 0.0)
        raise ModelError.new("blocco :#{name}: x0 deve avere #{n} elementi (ricevuti #{@x0.size})") unless @x0.size == n
        @feedthrough = @d.any? { |row| row.any? { |v| v != 0.0 } }
        @label = label || "ẋ=Ax+Bu"
      end

      def state_space : CrySpace::StateSpace?
        @sys
      end

      # State names are exposed as extra output ports (appended after the
      # real C x + D u outputs) so they flow through the same logging path
      # as any other signal — see Model#collect_log_entries.
      def state_ports : Hash(Symbol, Int32)?
        return nil unless names = @state_names
        h = {} of Symbol => Int32
        names.each_with_index { |name, i| h[name] = @n_real_outputs + i }
        h
      end

      private def unpack(t : Float64Tensor, rows : Int32, cols : Int32) : Array(Array(Float64))
        Array.new(rows) do |i|
          Array.new(cols) do |j|
            rows == 0 || cols == 0 ? 0.0 : t[i, j].value
          end
        end
      end

      def direct_feedthrough? : Bool
        @feedthrough
      end

      def state_size : Int32
        @n_states
      end

      def initial_state : Array(Float64)
        @x0.dup
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        @n_real_outputs.times do |i|
          acc = 0.0
          @n_states.times { |j| acc += @c[i][j] * x[j] }
          n_inputs.times { |j| acc += @d[i][j] * u[j] }
          y[i] = acc
        end
        if names = @state_names
          names.each_with_index { |_, i| y[@n_real_outputs + i] = x[i] }
        end
      end

      def derivative(t : Float64, x : Array(Float64), u : Array(Float64), dx : Array(Float64))
        @n_states.times do |i|
          acc = 0.0
          @n_states.times { |j| acc += @a[i][j] * x[j] }
          n_inputs.times { |j| acc += @b[i][j] * u[j] }
          dx[i] = acc
        end
      end

      def category : Symbol
        :continuous
      end

      def glyph_label : String
        @label
      end

      def params_description : String
        outputs_desc = @output_names.try(&.to_s) || n_outputs.to_s
        states_desc = @state_names.try(&.to_s) || @n_states.to_s
        "states: #{states_desc}, inputs: #{n_inputs}, outputs: #{outputs_desc}"
      end
    end

    # Discrete LTI state-space block: y[k] = Cx[k] + Du[k], with the state
    # advancing as a sampled difference equation x[k+1] = Ax[k] + Bu[k]
    # instead of a continuous derivative. Built from a CrySpace::StateSpace
    # whose dt is set (e.g. the result of StateSpace#sample(dt, method:)).
    #
    # Uses two state buffers: `@x` (x[k], read by `output` and frozen for
    # the whole macro step, including every RK4 substep) and `@x_next`
    # (x[k+1], staged by `update_sample` from the frozen `@x`). `@x_next`
    # is only promoted into `@x` by `commit_sample`, once per step, after
    # everything else in that step has already read `@x` — this is what
    # keeps the one-step delay intact instead of leaking x[k+1] back into
    # the step that computed it (which PID/Noise don't need to worry
    # about, since their sampled value directly *is* the step's output).
    class DiscreteStateSpaceBlock < Block
      getter n_states : Int32
      getter sys : CrySpace::StateSpace
      @a : Array(Array(Float64))
      @b : Array(Array(Float64))
      @c : Array(Array(Float64))
      @d : Array(Array(Float64))
      @x0 : Array(Float64)
      # Two persistent, pre-allocated buffers swapped between "current"
      # and "next" roles by commit_sample (see #cur/#nxt) — no per-step
      # Array allocation in the hot path, and no risk of the aliasing bug
      # a single shared buffer would cause (writing element i of x[k+1]
      # before x[k+1]'s computation for element i+1 has finished reading
      # the still-needed old x[k][i]).
      @buf_a : Array(Float64)
      @buf_b : Array(Float64)
      @a_is_current : Bool = true
      # commit_sample is called by the engine every step regardless of
      # rate_steps, but must only swap on a step where update_sample
      # actually staged fresh data into `nxt` — otherwise (multi-rate,
      # not due this step) it would swap onto whatever `nxt` was left
      # over from the *previous* due update instead of leaving `cur`
      # alone. See v0.3's "buffer pre-allocati" audit.
      @pending_commit : Bool = false
      @feedthrough : Bool
      @label : String
      @n_real_outputs : Int32
      @state_names : Array(Symbol)?
      @output_names : Array(Symbol)?
      @sys_dt : Float64
      @rate_steps : Int32 = 1

      def initialize(name : String, ss : CrySpace::StateSpace, x0 : Array(Float64)? = nil, label : String? = nil,
                     state_names : Array(Symbol)? = nil, output_names : Array(Symbol)? = nil)
        @sys_dt = ss.dt || raise ModelError.new("blocco :#{name}: dss richiede un CrySpace::StateSpace con dt impostato")
        n = ss.a.shape[0]
        m = ss.b.shape.size > 1 ? ss.b.shape[1] : 1
        p = ss.c.shape[0]
        if state_names && state_names.size != n
          raise ModelError.new("blocco :#{name}: state_names deve avere #{n} elementi (ricevuti #{state_names.size})")
        end
        if output_names && output_names.size != p
          raise ModelError.new("blocco :#{name}: output_names deve avere #{p} elementi (ricevuti #{output_names.size})")
        end
        super(name, m, p + (state_names.try(&.size) || 0))
        @sys = ss
        @n_states = n
        @n_real_outputs = p
        @state_names = state_names
        @output_names = output_names
        @a = unpack(ss.a, n, n)
        @b = unpack(ss.b, n, m)
        @c = unpack(ss.c, p, n)
        @d = unpack(ss.d, p, m)
        @x0 = x0 || Array(Float64).new(n, 0.0)
        raise ModelError.new("blocco :#{name}: x0 deve avere #{n} elementi (ricevuti #{@x0.size})") unless @x0.size == n
        @buf_a = @x0.dup
        @buf_b = @x0.dup
        @feedthrough = @d.any? { |row| row.any? { |v| v != 0.0 } }
        @label = label || "x[k+1]=Ax+Bu"
      end

      # x[k], frozen for the whole macro step (see the class comment).
      private def cur : Array(Float64)
        @a_is_current ? @buf_a : @buf_b
      end

      # x[k+1], staged by update_sample into the currently-inactive buffer.
      private def nxt : Array(Float64)
        @a_is_current ? @buf_b : @buf_a
      end

      def state_space : CrySpace::StateSpace?
        @sys
      end

      def state_ports : Hash(Symbol, Int32)?
        return nil unless names = @state_names
        h = {} of Symbol => Int32
        names.each_with_index { |name, i| h[name] = @n_real_outputs + i }
        h
      end

      private def unpack(t : Float64Tensor, rows : Int32, cols : Int32) : Array(Array(Float64))
        Array.new(rows) do |i|
          Array.new(cols) do |j|
            rows == 0 || cols == 0 ? 0.0 : t[i, j].value
          end
        end
      end

      # Only a nonzero D makes y[k] instantaneously depend on u[k] (a real
      # algebraic dependency the eval-order/loop-detection must see, exactly
      # as for the continuous StateSpaceBlock). When D == 0 — the common
      # case for a discretized strictly-proper system — the block has no
      # feedthrough and, like PID/Noise, breaks algebraic loops through it.
      def direct_feedthrough? : Bool
        @feedthrough
      end

      def sampled? : Bool
        true
      end

      def prepare(dt : Float64)
        @n_states.times do |i|
          @buf_a[i] = @x0[i]
          @buf_b[i] = @x0[i]
        end
        @a_is_current = true
        @pending_commit = false
        ratio = @sys_dt / dt
        steps = ratio.round.to_i
        unless steps >= 1 && (ratio - steps).abs < 1e-6 * {ratio.abs, 1.0}.max
          raise ModelError.new(
            "blocco :#{name}: il dt del sistema discreto (#{@sys_dt}) non è un multiplo intero " \
            "del dt del solver (#{dt}) — v0.3 supporta solo multi-rate a rapporto intero")
        end
        @rate_steps = steps
      end

      def rate_steps : Int32
        @rate_steps
      end

      def update_sample(t : Float64, u : Array(Float64), dt : Float64)
        c = cur
        n = nxt
        @n_states.times do |i|
          acc = 0.0
          @n_states.times { |j| acc += @a[i][j] * c[j] }
          n_inputs.times { |j| acc += @b[i][j] * u[j] }
          n[i] = acc
        end
        @pending_commit = true
      end

      def commit_sample(dt : Float64)
        return unless @pending_commit
        @a_is_current = !@a_is_current
        @pending_commit = false
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        c = cur
        @n_real_outputs.times do |i|
          acc = 0.0
          @n_states.times { |j| acc += @c[i][j] * c[j] }
          n_inputs.times { |j| acc += @d[i][j] * u[j] }
          y[i] = acc
        end
        if names = @state_names
          names.each_with_index { |_, i| y[@n_real_outputs + i] = c[i] }
        end
      end

      def category : Symbol
        :continuous
      end

      def glyph_label : String
        @label
      end

      def params_description : String
        outputs_desc = @output_names.try(&.to_s) || @n_real_outputs.to_s
        states_desc = @state_names.try(&.to_s) || @n_states.to_s
        "states: #{states_desc}, inputs: #{n_inputs}, outputs: #{outputs_desc} (discrete)"
      end
    end

    # SISO transfer function block: converted once to state-space via
    # CrySpace::TransferFunction#to_statespace (control canonical form).
    class TransferFcn < StateSpaceBlock
      @num : Array(Float64)
      @den : Array(Float64)

      def initialize(name : String, num : Array(Float64) | Array(Int32), den : Array(Float64) | Array(Int32))
        num_f = num.map(&.to_f64)
        den_f = den.map(&.to_f64)
        raise ArgumentError.new("tf denominator cannot be empty") if den_f.empty?
        raise ArgumentError.new("tf numerator order cannot exceed denominator order") if num_f.size > den_f.size
        tf = CrySpace::TransferFunction.new(num_f.to_tensor, den_f.to_tensor)
        @num = num_f
        @den = den_f
        super(name, tf.to_statespace, label: fraction_label)
      end

      private def fraction_label : String
        "#{poly_to_s(@num)} / #{poly_to_s(@den)}"
      end

      private def poly_to_s(coeffs : Array(Float64)) : String
        n = coeffs.size - 1
        terms = [] of String
        coeffs.each_with_index do |c, i|
          next if c == 0.0
          pow = n - i
          coef = (c == 1.0 && pow > 0) ? "" : format_num(c)
          terms << case pow
          when 0 then format_num(c)
          when 1 then "#{coef}s"
          else        "#{coef}s^#{pow}"
          end
        end
        terms.empty? ? "0" : terms.join(" + ")
      end

      private def format_num(v : Float64) : String
        v == v.to_i ? v.to_i.to_s : v.to_s
      end

      def params_description : String
        "num: #{@num}, den: #{@den}"
      end
    end

    # PID controller wrapping CrySpace::PIDController (filtered derivative,
    # clamping anti-windup). Discrete by nature: it is treated as a sampled
    # block updated once per solver step, its output held during RK4
    # substeps (ZOH). This also breaks algebraic loops through the
    # controller, as in Simulink.
    class PID < Block
      # `rate:` runs the controller at a slower rate than the base solver
      # step (e.g. `rate: 10` for a 10x slower outer loop) — an integer
      # multiplier, per v0.3's "simple multi-rate" (see Block#rate_steps).
      def initialize(name : String, kp : Number = 1.0, ki : Number = 0.0, kd : Number = 0.0,
                     filter_tf : Number = 0.01,
                     u_min : Number = -Float64::INFINITY, u_max : Number = Float64::INFINITY,
                     rate : Int32 = 1)
        super(name, 1, 1)
        @kp = kp.to_f64
        @ki = ki.to_f64
        @kd = kd.to_f64
        @pid = CrySpace::PIDController.new(@kp, @ki, @kd, filter_tf.to_f64, u_min.to_f64, u_max.to_f64)
        @u_held = 0.0
        raise ModelError.new("blocco :#{name}: rate deve essere >= 1") if rate < 1
        @rate = rate
      end

      def direct_feedthrough? : Bool
        false
      end

      def sampled? : Bool
        true
      end

      def rate_steps : Int32
        @rate
      end

      def prepare(dt : Float64)
        @pid.reset
        @u_held = 0.0
      end

      def update_sample(t : Float64, u : Array(Float64), dt : Float64)
        @u_held = @pid.update(u[0], dt)
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        y[0] = @u_held
      end

      def category : Symbol
        :continuous
      end

      def glyph_label : String
        "PID"
      end

      def params_description : String
        "kp: #{@kp}, ki: #{@ki}, kd: #{@kd}"
      end
    end

    # Rate Limiter: clamps how fast the output can rise/fall per unit
    # time, regardless of how quickly the input moves — a slew-rate
    # limit (actuator physical limits, avoiding a step reference from
    # commanding an instantaneous jump). Sampled at the base rate, same
    # two-buffer commit pattern as UnitDelay/DiscreteStateSpaceBlock:
    # y[k] = y[k-1] + clamp(u[k] - y[k-1], falling_rate*dt, rising_rate*dt).
    class RateLimiter < Block
      @rising_rate : Float64
      @falling_rate : Float64
      @x0 : Float64
      @held : Float64
      @next_held : Float64

      def initialize(name : String, rising_rate : Number = 1.0, falling_rate : Float64 | Int32 | Nil = nil, x0 : Number = 0.0)
        super(name, 1, 1)
        @rising_rate = rising_rate.to_f64
        @falling_rate = (falling_rate || -@rising_rate).to_f64
        raise ModelError.new("blocco :#{name}: rising_rate deve essere positivo") unless @rising_rate > 0.0
        raise ModelError.new("blocco :#{name}: falling_rate deve essere negativo") unless @falling_rate < 0.0
        @x0 = x0.to_f64
        @held = @x0
        @next_held = @x0
      end

      def direct_feedthrough? : Bool
        false
      end

      def sampled? : Bool
        true
      end

      def prepare(dt : Float64)
        @held = @x0
        @next_held = @x0
      end

      def update_sample(t : Float64, u : Array(Float64), dt : Float64)
        delta = (u[0] - @held).clamp(@falling_rate * dt, @rising_rate * dt)
        @next_held = @held + delta
      end

      def commit_sample(dt : Float64)
        @held = @next_held
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        y[0] = @held
      end

      def category : Symbol
        :continuous
      end

      def glyph_label : String
        "RL"
      end

      def params_description : String
        "rising: #{@rising_rate}/s, falling: #{@falling_rate}/s"
      end
    end
  end
end
