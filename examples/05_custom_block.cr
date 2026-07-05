require "../src/crysim"

# How to extend CrySim beyond the built-in DSL blocks: subclass
# CrySim::Block (the S-function equivalent, see src/crysim/block.cr) and
# register an instance with the `block` escape hatch inside the model.
#
# This models a mass sliding on a surface with Coulomb-like friction: a
# constant driving force is opposed by a friction force that always points
# against the direction of motion. The friction curve is smoothed with
# tanh (a common trick to avoid a hard discontinuity at zero velocity,
# which would make the ODE solver's job much harder).
class CoulombFriction < CrySim::Block
  def initialize(name : Symbol, @mu_n : Float64, @steepness : Float64 = 50.0)
    # 1 input (velocity), 1 output (friction force opposing it). Block's
    # own identity is a String (see block.cr) — .to_s is always safe going
    # this direction, unlike trying to synthesize a new Symbol.
    super(name.to_s, 1, 1)
  end

  def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
    velocity = u[0]
    y[0] = -@mu_n * Math.tanh(velocity * @steepness)
  end

  # metadata used by the SVG diagram renderer (optional, but nice to have)
  def glyph_label : String
    "friction"
  end

  def params_description : String
    "mu_n: #{@mu_n}, steepness: #{@steepness}"
  end
end

model = CrySim.model "sliding_mass_with_friction" do
  duration 6.0
  dt 0.001

  constant :drive_force, value: 2.0
  sum :net_force, signs: "++"
  block CoulombFriction.new(:friction, mu_n: 2.5) # > drive_force, so a terminal velocity exists
  integrator :velocity, x0: 0.0
  integrator :position, x0: 0.0
  scope :out, title: "Sliding mass with Coulomb friction"

  # net_force = drive_force + friction(velocity); friction reads velocity
  # from the same integrator it feeds into — a direct-feedthrough-free
  # loop (the integrator holds the state), so this is not an algebraic loop.
  connect :drive_force, to: {:net_force, 0}
  connect :friction, to: {:net_force, 1}
  connect :net_force, to: :velocity
  connect :velocity, to: :friction
  connect :velocity, to: :position

  connect :velocity, to: :out, as: :velocity
  connect :position, to: :out, as: :position
end

result = model.run
v = result[:velocity]
puts "terminal velocity : #{v.last.round(4)} m/s (net force balances friction)"
puts "final position    : #{result[:position].last.round(3)} m"

result.plot("sliding_mass.html")
model.render("sliding_mass_diagram.html")
puts "written: sliding_mass.html, sliding_mass_diagram.html"
