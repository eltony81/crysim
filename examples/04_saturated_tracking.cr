require "../src/crysim"

# Position tracking loop with two nonlinearities that never show up in a
# purely linear analysis: an actuator that clips at +/-3.0 (Saturation) and
# a sensor with a small dead zone around zero (DeadZone, e.g. static
# friction/backlash in a real position sensor).
#
# The reference step is deliberately large (10.0) relative to the actuator
# limit, so the controller saturates hard during the initial transient —
# this is where a naive PID would windup its integral term and overshoot
# badly. CrySpace::PIDController already includes clamping anti-windup
# (see src/crysim/blocks/continuous.cr), so the point of this example is to
# demonstrate that the anti-windup is actually earning its keep here.

model = CrySim.model "saturated_tracking" do
  duration 8.0
  dt 0.001
  method :rk4

  step :ref, amplitude: 10.0, start_time: 0.1
  sum :err, signs: "+-"
  pid :ctrl, kp: 4.0, ki: 2.0, kd: 0.3, u_min: -3.0, u_max: 3.0
  probe :u_cmd # inline monitor: see the controller command before it hits the actuator limit
  saturation :actuator, min: -3.0, max: 3.0
  tf :plant, num: [1.0], den: [1.0, 0.5, 0.0] # integrator-like plant with light damping
  deadzone :sensor_deadzone, threshold: 0.05
  scope :out, title: "Saturated position tracking"

  connect :ref, to: {:err, 0}
  connect :sensor_deadzone, to: {:err, 1}
  connect :err, to: :ctrl
  connect :ctrl, to: :u_cmd
  connect :u_cmd, to: :actuator
  connect :actuator, to: :plant
  connect :plant, to: :sensor_deadzone
  connect :plant, to: :out, as: :position
  connect :ref, to: :out
end

result = model.run
pos = result[:position]
cmd = result[:u_cmd]

overshoot = (pos.max - 10.0) / 10.0 * 100.0
saturated_fraction = cmd.count { |v| v.abs >= 2.999 } / cmd.size.to_f * 100.0

puts "final position    : #{pos.last.round(3)} (target 10.0)"
puts "overshoot         : #{overshoot.round(1)}%"
puts "time saturated    : #{saturated_fraction.round(1)}% of the run (u_cmd at the +/-3.0 limit)"

result.plot("saturated_tracking.html")
model.render("saturated_tracking_diagram.html")
puts "written: saturated_tracking.html, saturated_tracking_diagram.html"
