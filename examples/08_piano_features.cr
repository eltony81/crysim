require "../src/crysim"

# Tour of the robustness/block-library additions from PIANO_FEATURE.md:
# NaN/Inf detection with context, lookup_table, and rate_limiter.

# ---------------------------------------------------------------------------
# 1. lookup_table: a nonlinear valve characteristic (0 command -> 0 flow,
#    saturating past half-open) driven by a ramping command signal.
valve_model = CrySim.model "valve_curve" do
  duration 0.1
  dt 0.01
  ramp :command, slope: 10.0 # 0 .. 1.0 over the run
  lookup_table :valve, breakpoints: [0.0, 0.25, 0.5, 1.0], values: [0.0, 20.0, 90.0, 100.0]
  scope :out
  connect :command, to: :valve
  connect :command, to: :out, as: :command
  connect :valve, to: :out, as: :flow
end
vr = valve_model.run
puts "-- lookup_table --"
puts "command: #{vr[:command].map(&.round(2))}"
puts "flow    : #{vr[:flow].map(&.round(1))}"

# ---------------------------------------------------------------------------
# 2. rate_limiter: an actuator that physically can't jump instantly —
#    a step reference gets slewed at a fixed rate instead.
rl_model = CrySim.model "actuator_slew" do
  duration 1.0
  dt 0.1
  step :setpoint, amplitude: 10.0
  rate_limiter :actuator, rising_rate: 4.0, falling_rate: -8.0 # opens slower than it closes
  scope :out
  connect :setpoint, to: :actuator
  connect :actuator, to: :out, as: :position
end
rlr = rl_model.run
puts
puts "-- rate_limiter --"
puts "position: #{rlr[:position].map(&.round(2))}"

# ---------------------------------------------------------------------------
# 3. NaN/Inf detection with context: a custom expression hits a
#    singularity partway through the run. Instead of a silently broken
#    plot, CrySim fails fast naming the exact block/port/time, and the
#    diagram can highlight the culprit for a quick visual diagnosis.
broken_model = CrySim.model "broken_expr" do
  duration 0.05
  dt 0.01
  ramp :u, slope: 1.0
  fn :risky, expr: "1/(u-0.02)" # singularity at t=0.02
  scope :out
  connect :u, to: :risky
  connect :risky, to: :out, as: :y
end

puts
puts "-- NaN/Inf detection --"
begin
  broken_model.run
rescue err : CrySim::NonFiniteValueError
  puts "caught: #{err.message}"
  broken_model.render_error(err, "broken_expr_diagram.html")
  puts "diagram written with :#{err.block_name} highlighted in red: broken_expr_diagram.html"
end
