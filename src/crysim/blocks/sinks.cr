module CrySim
  module Blocks
    # Inline pass-through monitor (the Simulink test-point equivalent):
    # y = u, zero state, zero cost. Its signal is logged automatically
    # under the probe name. Adding or removing a probe never changes the
    # numerical results of the simulation.
    class Probe < Block
      def initialize(name : String)
        super(name, 1, 1)
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
        y[0] = u[0]
      end

      def auto_log? : Bool
        true
      end

      def category : Symbol
        :probe
      end

      def glyph_label : String
        "~"
      end

      def params_description : String
        "pass-through monitor (y = u)"
      end
    end

    # Terminal sink: every incoming wire is logged (under the wire label,
    # or the source block name) and plotted by SimResult#plot. Accepts any
    # number of incoming connections; each one appends an input port.
    class Scope < Block
      getter title : String

      def initialize(name : String, title : String? = nil)
        super(name, 0, 0)
        @title = title || name.to_s
      end

      def elastic_inputs? : Bool
        true
      end

      def direct_feedthrough? : Bool
        false
      end

      def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
      end

      def category : Symbol
        :sink
      end

      def glyph_label : String
        "Scope"
      end

      def params_description : String
        "title: #{@title}"
      end
    end
  end
end
