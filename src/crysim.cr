require "cryspace"
require "eeeval"

require "./crysim/block"
require "./crysim/blocks/sources"
require "./crysim/blocks/math"
require "./crysim/blocks/continuous"
require "./crysim/blocks/sinks"
require "./crysim/model"
require "./crysim/subsystem"
require "./crysim/builder"
require "./crysim/engine"
require "./crysim/result"
require "./crysim/diagram/svg_renderer"

{% if flag?(:arrow) %}
  require "./crysim/arrow_io"
{% end %}

# CrySim — a Simulink-inspired block-diagram simulation library.
#
# Compose a model with the DSL, run it, plot it:
#
#   model = CrySim.model "example" do
#     duration 5.0
#     dt 0.001
#     step :ref, amplitude: 1.0
#     tf :plant, num: [1.0], den: [0.5, 1.0]
#     scope :out
#     connect :ref, to: :plant
#     connect :plant, to: :out, as: :y
#   end
#
#   result = model.run
#   result.plot("response.html")
#   model.render("diagram.html")
module CrySim
  VERSION = "0.4.0"

  def self.model(name : String, &) : Model
    builder = ModelBuilder.new(name)
    with builder yield
    builder.build
  end
end
