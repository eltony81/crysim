module CrySim
  module Blocks
    class Gain < Block
      getter k : Float64

      def initialize(name : String, k : Number = 1.0)
        super(name, 1, 1)
        @k = k.to_f64
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        y[0] = @k * u[0]
      end

      def category : Symbol
        :gain
      end

      def glyph_label : String
        @k.to_s
      end

      def params_description : String
        "k: #{@k}"
      end
    end

    # N-input sum with per-input sign, declared as a string: sum :e, signs: "+-"
    class Sum < Block
      getter signs : String
      @coeffs : Array(Float64)

      def initialize(name : String, signs : String = "++")
        raise ArgumentError.new("sum signs must contain only '+' and '-'") unless signs.chars.all? { |c| c == '+' || c == '-' }
        raise ArgumentError.new("sum needs at least one input") if signs.empty?
        super(name, signs.size, 1)
        @signs = signs
        @coeffs = signs.chars.map { |c| c == '+' ? 1.0 : -1.0 }
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        acc = 0.0
        @coeffs.each_with_index { |c, i| acc += c * u[i] }
        y[0] = acc
      end

      def category : Symbol
        :sum
      end

      def glyph_label : String
        @signs
      end

      def params_description : String
        "signs: #{@signs}"
      end
    end

    class Product < Block
      def initialize(name : String, n_inputs : Int32 = 2)
        raise ArgumentError.new("product needs at least one input") if n_inputs < 1
        super(name, n_inputs, 1)
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        acc = 1.0
        n_inputs.times { |i| acc *= u[i] }
        y[0] = acc
      end

      def glyph_label : String
        "×"
      end

      def params_description : String
        "n_inputs: #{n_inputs}"
      end
    end

    class Saturation < Block
      def initialize(name : String, min : Number = -1.0, max : Number = 1.0)
        super(name, 1, 1)
        @min = min.to_f64
        @max = max.to_f64
        raise ArgumentError.new("saturation min must be < max") unless @min < @max
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        y[0] = u[0].clamp(@min, @max)
      end

      def glyph_label : String
        "Sat"
      end

      def params_description : String
        "min: #{@min}, max: #{@max}"
      end
    end

    class DeadZone < Block
      def initialize(name : String, threshold : Number = 0.5)
        super(name, 1, 1)
        @threshold = threshold.to_f64.abs
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        v = u[0]
        y[0] = if v > @threshold
                 v - @threshold
               elsif v < -@threshold
                 v + @threshold
               else
                 0.0
               end
      end

      def glyph_label : String
        "DZ"
      end

      def params_description : String
        "threshold: #{@threshold}"
      end
    end

    # Arbitrary transformation from a Crystal Proc:
    #   fn :sq, ->(u : Float64, t : Float64) { u * u }
    class Fn < Block
      def initialize(name : String, @fn : Proc(Float64, Float64, Float64))
        super(name, 1, 1)
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        y[0] = @fn.call(u[0], t)
      end

      def glyph_label : String
        "f(u,t)"
      end

      def params_description : String
        "custom Proc"
      end
    end

    # Arbitrary transformation from an eeeval expression of u and t:
    #   fn :sq, expr: "u^2"
    class ExprFn < Block
      @ast : EEEval::AST::Node
      @env : Hash(String, Float64)

      def initialize(name : String, expr : String)
        super(name, 1, 1)
        @expr = expr
        @ast = EEEval::CalcFuncParser.compile(expr)
        @env = EEEval::Constants::DEFAULT_ENV.dup
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        @env["t"] = t
        @env["u"] = u[0]
        y[0] = @ast.evaluate(@env)
      end

      def glyph_label : String
        "f(u,t)"
      end

      def params_description : String
        "expr: #{@expr}"
      end
    end

    # Switch/If: 3 ports (0: data if the condition holds, 1: control,
    # 2: data otherwise) — the Simulink Switch layout. Two ways to state
    # the condition:
    #
    #   - `criteria:`/`threshold:` (default): a plain relational test in
    #     Crystal — the common "switch above/below a threshold" case.
    #   - `condition:`: an EEEval::CondParser boolean expression, with the
    #     control value sprintf-substituted into it (`"%.6f == 1.0"`).
    #     CondParser only supports `==`/`!=`/`&&`/`||` (no `>`/`<` — its
    #     tokenizer doesn't recognize those characters at all), so this
    #     mode is for exact-value / discrete-mode gating (e.g. control ==
    #     an enum-like flag), not threshold switching — use `criteria:`
    #     for that instead.
    class Switch < Block
      VALID_CRITERIA = {:greater_than, :greater_equal, :less_than, :less_equal, :equal_to}

      def initialize(name : String, criteria : Symbol = :greater_than, threshold : Number = 0.0,
                     condition : String? = nil)
        super(name, 3, 1)
        if condition
          @condition = condition
        else
          unless VALID_CRITERIA.includes?(criteria)
            raise ModelError.new("blocco :#{name}: criteria deve essere uno tra #{VALID_CRITERIA.join(", ")}")
          end
          @condition = nil
        end
        @criteria = criteria
        @threshold = threshold.to_f64
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        control = u[1]
        take_true =
          if cond = @condition
            EEEval::CondParser.evaluate(cond % control)
          else
            case @criteria
            when :greater_than  then control > @threshold
            when :greater_equal then control >= @threshold
            when :less_than     then control < @threshold
            when :less_equal    then control <= @threshold
            else                     control == @threshold # :equal_to
            end
          end
        y[0] = take_true ? u[0] : u[2]
      end

      def direct_feedthrough? : Bool
        true
      end

      def glyph_label : String
        "SW"
      end

      def params_description : String
        @condition ? "condition: #{@condition}" : "criteria: #{@criteria}, threshold: #{@threshold}"
      end
    end

    # 1D lookup table: piecewise-linear interpolation over a breakpoint/
    # value table. Outside the table, `extrapolate:` picks between holding
    # the nearest edge value (default, matches Simulink's default too) or
    # continuing the edge segment's slope.
    class LookupTable1D < Block
      @breakpoints : Array(Float64)
      @values : Array(Float64)

      def initialize(name : String, breakpoints : Array(Float64) | Array(Int32),
                     values : Array(Float64) | Array(Int32), extrapolate : Bool = false)
        super(name, 1, 1)
        bp = breakpoints.map(&.to_f64)
        vals = values.map(&.to_f64)
        raise ModelError.new("blocco :#{name}: breakpoints e values devono avere la stessa lunghezza") unless bp.size == vals.size
        raise ModelError.new("blocco :#{name}: servono almeno 2 punti") if bp.size < 2
        unless (0...bp.size - 1).all? { |i| bp[i] < bp[i + 1] }
          raise ModelError.new("blocco :#{name}: breakpoints devono essere strettamente crescenti")
        end
        @breakpoints = bp
        @values = vals
        @extrapolate = extrapolate
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        xv = u[0]
        bp = @breakpoints
        vals = @values
        n = bp.size

        if xv <= bp[0]
          y[0] = @extrapolate ? lerp(bp[0], vals[0], bp[1], vals[1], xv) : vals[0]
          return
        end
        if xv >= bp[n - 1]
          y[0] = @extrapolate ? lerp(bp[n - 2], vals[n - 2], bp[n - 1], vals[n - 1], xv) : vals[n - 1]
          return
        end
        (n - 1).times do |i|
          next unless xv >= bp[i] && xv <= bp[i + 1]
          y[0] = lerp(bp[i], vals[i], bp[i + 1], vals[i + 1], xv)
          return
        end
      end

      private def lerp(x0 : Float64, y0 : Float64, x1 : Float64, y1 : Float64, x : Float64) : Float64
        y0 + (x - x0) * (y1 - y0) / (x1 - x0)
      end

      def glyph_label : String
        "LUT"
      end

      def params_description : String
        "#{@breakpoints.size} points, extrapolate: #{@extrapolate}"
      end
    end
  end
end
