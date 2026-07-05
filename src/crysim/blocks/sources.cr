module CrySim
  module Blocks
    # Base for all sources: no inputs, one output, pure function of time.
    abstract class Source < Block
      def initialize(name : String)
        super(name, 0, 1)
      end

      def direct_feedthrough? : Bool
        false
      end

      def category : Symbol
        :source
      end

      abstract def value(t : Float64) : Float64

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        y[0] = value(t)
      end
    end

    class Constant < Source
      def initialize(name : String, value : Number = 1.0)
        super(name)
        @value = value.to_f64
      end

      def value(t : Float64) : Float64
        @value
      end

      def glyph_label : String
        @value.to_s
      end

      def params_description : String
        "value: #{@value}"
      end
    end

    class Step < Source
      def initialize(name : String, amplitude : Number = 1.0, start_time : Number = 0.0, offset : Number = 0.0)
        super(name)
        @amplitude = amplitude.to_f64
        @start_time = start_time.to_f64
        @offset = offset.to_f64
      end

      def value(t : Float64) : Float64
        t >= @start_time ? @offset + @amplitude : @offset
      end

      def glyph_label : String
        "Step"
      end

      def params_description : String
        "amplitude: #{@amplitude}, start_time: #{@start_time}"
      end
    end

    class Ramp < Source
      def initialize(name : String, slope : Number = 1.0, start_time : Number = 0.0)
        super(name)
        @slope = slope.to_f64
        @start_time = start_time.to_f64
      end

      def value(t : Float64) : Float64
        t >= @start_time ? @slope * (t - @start_time) : 0.0
      end

      def glyph_label : String
        "Ramp"
      end

      def params_description : String
        "slope: #{@slope}, start_time: #{@start_time}"
      end
    end

    class Sine < Source
      def initialize(name : String, amplitude : Number = 1.0, freq : Number = 1.0, phase : Number = 0.0, offset : Number = 0.0)
        super(name)
        @amplitude = amplitude.to_f64
        @freq = freq.to_f64
        @phase = phase.to_f64
        @offset = offset.to_f64
      end

      def value(t : Float64) : Float64
        @offset + @amplitude * Math.sin(2.0 * Math::PI * @freq * t + @phase)
      end

      def glyph_label : String
        "Sine"
      end

      def params_description : String
        "amplitude: #{@amplitude}, freq: #{@freq} Hz, phase: #{@phase}"
      end
    end

    class Cosine < Source
      def initialize(name : String, amplitude : Number = 1.0, freq : Number = 1.0, phase : Number = 0.0, offset : Number = 0.0)
        super(name)
        @amplitude = amplitude.to_f64
        @freq = freq.to_f64
        @phase = phase.to_f64
        @offset = offset.to_f64
      end

      def value(t : Float64) : Float64
        @offset + @amplitude * Math.cos(2.0 * Math::PI * @freq * t + @phase)
      end

      def glyph_label : String
        "Cosine"
      end

      def params_description : String
        "amplitude: #{@amplitude}, freq: #{@freq} Hz, phase: #{@phase}"
      end
    end

    # Numerical Dirac impulse: a rectangular pulse lasting a single solver
    # step with amplitude area/dt (same convention as cryspace
    # impulse_response).
    #
    # Method matters here more than for any other source. RK4 evaluates
    # the derivative at t, t+dt/2 (twice), and t+dt; that last point (k4)
    # lands exactly on the pulse's excluded right edge (value(t) requires
    # t < time+dt), so k4 always sees the impulse as already off. The
    # weighted average (k1+2k2+2k3+k4)/6 then always undercounts the
    # impulse's momentum by exactly 1/6, regardless of dt — confirmed
    # empirically: a first-order system's RK4 impulse response converges
    # to 5/6 of the analytic value as dt -> 0, not to the analytic value
    # itself. `method: :euler` doesn't have this bias (it only ever
    # samples the interval's start, which is correctly "on") and matches
    # the analytic response closely. Use `method: :euler` for impulse
    # response work; this is a fixed-step-solver-vs-discontinuous-input
    # limitation, not something a boundary-convention tweak fixes cleanly
    # (shifting which edge is inclusive just relocates the same bias into
    # the neighboring step) — a proper fix needs event-based impulse
    # handling (PIANO_FEATURE.md area C, deferred).
    class Impulse < Source
      def initialize(name : String, area : Number = 1.0, time : Number = 0.0)
        super(name)
        @area = area.to_f64
        @time = time.to_f64
        @dt = 0.0
      end

      def prepare(dt : Float64)
        @dt = dt
      end

      def value(t : Float64) : Float64
        return 0.0 if @dt == 0.0
        t >= @time && t < @time + @dt ? @area / @dt : 0.0
      end

      def glyph_label : String
        "Impulse"
      end

      def params_description : String
        "area: #{@area}, time: #{@time}"
      end
    end

    class Pulse < Source
      def initialize(name : String, amplitude : Number = 1.0, period : Number = 1.0, duty : Number = 0.5, offset : Number = 0.0)
        super(name)
        @amplitude = amplitude.to_f64
        @period = period.to_f64
        @duty = duty.to_f64
        @offset = offset.to_f64
        raise ArgumentError.new("pulse period must be positive") unless @period > 0.0
      end

      def value(t : Float64) : Float64
        phase = (t % @period) / @period
        phase < @duty ? @offset + @amplitude : @offset
      end

      def glyph_label : String
        "Pulse"
      end

      def params_description : String
        "amplitude: #{@amplitude}, period: #{@period}, duty: #{@duty}"
      end
    end

    class Sawtooth < Source
      def initialize(name : String, amplitude : Number = 1.0, period : Number = 1.0)
        super(name)
        @amplitude = amplitude.to_f64
        @period = period.to_f64
        raise ArgumentError.new("sawtooth period must be positive") unless @period > 0.0
      end

      def value(t : Float64) : Float64
        @amplitude * ((t % @period) / @period)
      end

      def glyph_label : String
        "Sawtooth"
      end

      def params_description : String
        "amplitude: #{@amplitude}, period: #{@period}"
      end
    end

    class Triangle < Source
      def initialize(name : String, amplitude : Number = 1.0, period : Number = 1.0)
        super(name)
        @amplitude = amplitude.to_f64
        @period = period.to_f64
        raise ArgumentError.new("triangle period must be positive") unless @period > 0.0
      end

      def value(t : Float64) : Float64
        phase = (t % @period) / @period
        phase < 0.5 ? @amplitude * 2.0 * phase : @amplitude * 2.0 * (1.0 - phase)
      end

      def glyph_label : String
        "Triangle"
      end

      def params_description : String
        "amplitude: #{@amplitude}, period: #{@period}"
      end
    end

    # Linear frequency sweep from f0 to f1 over [0, t1]; continues at f1
    # with continuous phase afterwards.
    class Chirp < Source
      def initialize(name : String, amplitude : Number = 1.0, f0 : Number = 0.1, f1 : Number = 10.0, t1 : Number = 10.0)
        super(name)
        @amplitude = amplitude.to_f64
        @f0 = f0.to_f64
        @f1 = f1.to_f64
        @t1 = t1.to_f64
        raise ArgumentError.new("chirp t1 must be positive") unless @t1 > 0.0
      end

      def value(t : Float64) : Float64
        if t <= @t1
          phase = 2.0 * Math::PI * (@f0 * t + (@f1 - @f0) * t * t / (2.0 * @t1))
        else
          phase_t1 = 2.0 * Math::PI * (@f0 * @t1 + (@f1 - @f0) * @t1 / 2.0)
          phase = phase_t1 + 2.0 * Math::PI * @f1 * (t - @t1)
        end
        @amplitude * Math.sin(phase)
      end

      def glyph_label : String
        "Chirp"
      end

      def params_description : String
        "f0: #{@f0} Hz, f1: #{@f1} Hz, t1: #{@t1}"
      end
    end

    # White gaussian noise, sampled once per macro step and held during
    # solver substeps (ZOH) so the integrator sees a reproducible signal.
    class Noise < Source
      def initialize(name : String, sigma : Number = 1.0, mean : Number = 0.0, seed : Int32? = nil)
        super(name)
        @sigma = sigma.to_f64
        @mean = mean.to_f64
        @rng = seed ? Random.new(seed.not_nil!) : Random.new
        @held = 0.0
        @spare = nil.as(Float64?)
      end

      def sampled? : Bool
        true
      end

      def update_sample(t : Float64, u : Array(Float64), dt : Float64)
        @held = @mean + @sigma * gaussian
      end

      def value(t : Float64) : Float64
        @held
      end

      # Box-Muller transform.
      private def gaussian : Float64
        if spare = @spare
          @spare = nil
          return spare
        end
        u1 = 0.0
        while u1 == 0.0
          u1 = @rng.next_float
        end
        u2 = @rng.next_float
        r = Math.sqrt(-2.0 * Math.log(u1))
        @spare = r * Math.sin(2.0 * Math::PI * u2)
        r * Math.cos(2.0 * Math::PI * u2)
      end

      def glyph_label : String
        "Noise"
      end

      def params_description : String
        "sigma: #{@sigma}, mean: #{@mean}"
      end
    end

    # Arbitrary source from a Crystal Proc: signal :src, ->(t : Float64) { ... }
    class SignalSource < Source
      def initialize(name : String, @fn : Proc(Float64, Float64))
        super(name)
      end

      def value(t : Float64) : Float64
        @fn.call(t)
      end

      def glyph_label : String
        "f(t)"
      end

      def params_description : String
        "custom Proc"
      end
    end

    # Arbitrary source from an eeeval expression of t, compiled once:
    # signal :src, expr: "0.5*t + 0.05*sin(2*pi*10*t)"
    class ExprSource < Source
      @ast : EEEval::AST::Node
      @env : Hash(String, Float64)

      def initialize(name : String, expr : String)
        super(name)
        @expr = expr
        @ast = EEEval::CalcFuncParser.compile(expr)
        @env = EEEval::Constants::DEFAULT_ENV.dup
      end

      def value(t : Float64) : Float64
        @env["t"] = t
        @ast.evaluate(@env)
      end

      def glyph_label : String
        "f(t)"
      end

      def params_description : String
        "expr: #{@expr}"
      end
    end
  end
end
