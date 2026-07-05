require "../src/crysim"

# Mass-spring-damper system defined directly in state-space (A, B, C, D),
# the primary way to bring an arbitrary linear system into CrySim without
# going through a transfer function.
#
# Physics: m*x'' + c*x' + k*x = F(t), with state vector x = [position, velocity].
#   A = [[0, 1], [-k/m, -c/m]]   B = [[0], [1/m]]   C = [[1, 0]]   D = [[0]]
#
# We pick m=1, k=20, c=1 (lightly damped, oscillatory) and drive it with a
# unit step force. `state_names:`/`output_names:` turn the raw state/output
# indices into readable labels: they are auto-logged into SimResult and
# shown on the SVG diagram tooltip, instead of a generic "states: 2".
m = 1.0
k = 20.0
c = 1.0

model = CrySim.model "mass_spring_damper" do
  duration 5.0
  dt 0.001

  step :force, amplitude: 1.0
  ss :plant, a: [[0.0, 1.0], [-k / m, -c / m]], b: [[0.0], [1.0 / m]],
             c: [[1.0, 0.0]], d: [[0.0]],
             state_names: [:position, :velocity], output_names: [:position]
  scope :out, title: "Mass-spring-damper response"

  connect :force, to: :plant
  connect :plant, to: :out, as: :position
end

result = model.run

# Cross-check: the same dynamics expressed as a transfer function should
# match the state-space simulation bit for bit (both go through cryspace's
# StateSpace machinery, just built two different ways).
tf_model = CrySim.model "mass_spring_damper_tf" do
  duration 5.0
  dt 0.001
  step :force, amplitude: 1.0
  # G(s) = 1 / (m*s^2 + c*s + k)
  tf :plant, num: [1.0], den: [m, c, k]
  scope :out
  connect :force, to: :plant
  connect :plant, to: :out, as: :position
end
tf_result = tf_model.run

max_diff = result[:position].map_with_index { |v, i| (v - tf_result[:position][i]).abs }.max
puts "peak position          : #{result[:position].max.round(4)} m"
puts "settle-time position   : #{result[:position].last.round(4)} m (expect F/k = #{(1.0 / k).round(4)})"
# `velocity` was never wired to a Scope; it is still logged automatically
# because it is a named state of the ss block.
puts "peak velocity           : #{result[:velocity].max.round(4)} m/s"
puts "max diff vs. tf model   : #{max_diff.round(10)} (should be ~0, same dynamics two ways)"

# Reach back into cryspace for analysis that has nothing to do with time
# simulation, without leaving the CrySim model.
poles = model.state_space_of(:plant).poles
puts "poles                   : #{poles.map { |p| p.round(3) }}"

result.plot("mass_spring_damper.html")
model.render("mass_spring_damper_diagram.html")
puts "written: mass_spring_damper.html, mass_spring_damper_diagram.html"

# ---------------------------------------------------------------------------
# Closed-loop position control: drive the SAME plant to an arbitrary
# setpoint instead of just letting the spring push it wherever a constant
# force happens to leave it. Open-loop, a unit force settles at F/k = 0.05 m
# (see above) — with feedback we can instead hit any target position we
# choose, and reject the spring's stiffness/damping instead of being at
# their mercy. A PID (already used in 01_pid_loop.cr) closes the loop:
# error = setpoint - position drives a force command back into the plant.
#
# Tuning note: kd is on the *error*, not the measurement (that's all
# CrySpace::PIDController offers today), so a step setpoint change causes a
# brief, large "derivative kick" in the force command right at t=0.1s —
# `filter_tf` (the derivative low-pass) tames it but can't remove it
# outright. This is a well-known real-world PID limitation, not a CrySim
# bug: production controllers usually switch to derivative-on-measurement
# to avoid it entirely, which is out of scope for this minimal PID.
setpoint_value = 1.0

closed_loop = CrySim.model "mass_spring_damper_closed_loop" do
  duration 10.0
  dt 0.001
  method :rk4

  step :setpoint, amplitude: setpoint_value, start_time: 0.1
  sum :err, signs: "+-"
  pid :ctrl, kp: 60.0, ki: 25.0, kd: 8.0, filter_tf: 0.03
  probe :u_cmd # inline monitor: the force command the controller sends to the plant
  ss :plant, a: [[0.0, 1.0], [-k / m, -c / m]], b: [[0.0], [1.0 / m]],
             c: [[1.0, 0.0]], d: [[0.0]],
             state_names: [:position, :velocity]
  scope :out, title: "Closed-loop position control"

  connect :setpoint, to: {:err, 0}
  connect :plant, to: {:err, 1} # feedback: plant's real output (position)
  connect :err, to: :ctrl
  connect :ctrl, to: :u_cmd
  connect :u_cmd, to: :plant
  connect :plant, to: :out, as: :position
  connect :setpoint, to: :out
end

cl_result = closed_loop.run
cl_pos = cl_result[:position]
overshoot = (cl_pos.max - setpoint_value) / setpoint_value * 100.0

puts
puts "-- closed-loop position control (setpoint = #{setpoint_value} m) --"
puts "final position          : #{cl_pos.last.round(4)} m (target #{setpoint_value}, still converging — light damping)"
puts "peak overshoot          : #{overshoot.round(1)}%"
puts "peak force command      : #{cl_result[:u_cmd].map(&.abs).max.round(2)} N (includes the derivative-kick spike at t=0.1s)"
puts "(open loop needed to guess the right constant force to land anywhere " \
     "near a target; closed loop hits it directly and rejects the spring)"

cl_result.plot("mass_spring_damper_closed_loop.html")
closed_loop.render("mass_spring_damper_closed_loop_diagram.html")
puts "written: mass_spring_damper_closed_loop.html, mass_spring_damper_closed_loop_diagram.html"
