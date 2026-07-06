require "../src/crysim"

# Fase 5 of PIANO_STATESPACE.md: parametric state-space construction.
# `ss` normally takes literal Float64 matrices. This adds an overload where
# every matrix entry is instead an eeeval expression string ("-k/m", "1/m",
# ...), compiled and evaluated once at build time against a `params:` hash
# — the same Hash(Symbol, Float64) that `use` already binds for parametric
# subsystems (v0.2). One template's matrices, many physical instances.

# ---------------------------------------------------------------------------
# 1. Standalone parametric `ss`, no subsystem involved: a spring-mass-damper
#    (ẍ = (F - c*ẋ - k*x)/m, i.e. a: [[0,1],[-k/m,-c/m]], b: [[0],[1/m]])
#    written once with symbolic matrix entries, bound to concrete k/m/c via
#    `params:`. Compared against the exact same system built the literal way
#    to confirm the expression path produces identical matrices.
k, m, c = 40.0, 2.0, 0.5

literal_model = CrySim.model "spring_mass_literal" do
  duration 2.0
  dt 0.001
  step :force, amplitude: 1.0
  ss :dynamics, a: [[0.0, 1.0], [-k/m, -c/m]], b: [[0.0], [1.0/m]],
     c: [[1.0, 0.0]], d: [[0.0]]
  scope :out
  connect :force, to: :dynamics
  connect :dynamics, to: :out, as: :position
end

parametric_model = CrySim.model "spring_mass_parametric" do
  duration 2.0
  dt 0.001
  step :force, amplitude: 1.0
  # Every cell is an eeeval expression string; k/m/c come from `params:`,
  # the named math constants (pi, e, ...) are available too if needed.
  ss :dynamics, a: [["0", "1"], ["-k/m", "-c/m"]], b: [["0"], ["1/m"]],
     c: [["1", "0"]], d: [["0"]],
     params: {k: 40.0, m: 2.0, c: 0.5}
  scope :out
  connect :force, to: :dynamics
  connect :dynamics, to: :out, as: :position
end

lit = literal_model.run
par = parametric_model.run
max_diff = lit[:position].zip(par[:position]).map { |a, b| (a - b).abs }.max

puts "-- standalone parametric ss --"
puts "literal    final position: #{lit[:position].last.round(4)}"
puts "parametric final position: #{par[:position].last.round(4)}"
puts "max diff vs literal build: #{max_diff} (expect 0.0 — same matrices, just built differently)"

# ---------------------------------------------------------------------------
# 2. The scenario Fase 5 was actually built for: a *subsystem template*
#    whose internal `ss` forwards the template's own `params` straight
#    through to the matrix expressions. Every `use` of the template gets
#    its own physical k/m/c without rewriting a single matrix by hand.
spring_mass_damper = CrySim.subsystem("spring_mass_damper") do |sub, params|
  sub.ss :dynamics, a: [["0", "1"], ["-k/m", "-c/m"]], b: [["0"], ["1/m"]],
         c: [["1", "0"]], d: [["0"]], params: params
  sub.inport  :force,    to: :dynamics
  sub.outport :position, from: :dynamics
end

two_oscillators = CrySim.model "two_oscillators" do
  duration 3.0
  dt 0.001
  step :f, amplitude: 1.0

  # Same template, two critically/near-critically damped instances with
  # different stiffness — each settles at its own steady-state F/k.
  use spring_mass_damper, as: :stiff, k: 100.0, m: 1.0, c: 20.0
  use spring_mass_damper, as: :soft,  k: 10.0,  m: 1.0, c: 6.0

  scope :out
  connect :f, to: :stiff
  connect :f, to: :soft
  connect :stiff, to: :out, as: :stiff_pos
  connect :soft,  to: :out, as: :soft_pos
end

result = two_oscillators.run
puts
puts "-- parametric subsystem (two instances, one template) --"
puts "stiff spring final position: #{result[:stiff_pos].last.round(4)} (expect 1/k = 0.01)"
puts "soft spring final position : #{result[:soft_pos].last.round(4)} (expect 1/k = 0.1)"

# ---------------------------------------------------------------------------
# 3. A typo'd or missing parameter is a build-time ModelError naming the
#    block and the offending expression, not a bare eeeval exception or a
#    silent wrong matrix.
puts
puts "-- undefined parameter in a matrix expression --"
begin
  CrySim.model "broken_params" do
    duration 0.1
    dt 0.01
    step :force
    ss :dynamics, a: [["0", "1"], ["-k/q", "-c/m"]], b: [["0"], ["1/m"]],
       c: [["1", "0"]], d: [["0"]],
       params: {k: 40.0, m: 2.0, c: 0.5} # "q" is never defined
  end
rescue err : CrySim::ModelError
  puts "caught: #{err.message}"
end

