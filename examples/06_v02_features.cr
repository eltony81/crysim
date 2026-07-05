require "../src/crysim"

# Showcases the v0.2 DSL additions together: subsystems (`use`), the `>>`
# chain sugar, the `feedback from:/to:` sugar, and the unified report
# (block diagram with a per-wire sparkline of the logged signal, followed
# by the same plot panels as `result.plot`, in one HTML file).
#
# Physical setup: a PID position controller drives two cascaded first-order
# lags (think: an actuator lag followed by a sensor lag) in a unity
# feedback loop. The two lags are two instances of the SAME reusable
# subsystem template, each given its own time constant.
lag_stage = CrySim.subsystem("first_order_lag") do |sub, params|
  sub.tf :dynamics, num: [1.0], den: [params[:tau], 1.0]
  sub.inport  :v_in,  to: :dynamics
  sub.outport :v_out, from: :dynamics
end

model = CrySim.model "cascaded_lags_with_pid" do
  duration 6.0
  dt 0.001

  step :setpoint, amplitude: 1.0, start_time: 0.1
  sum :err, signs: "+-"
  # >> chains real blocks (controller -> inline monitor). Subsystem
  # instances are virtual ports, not blocks, so they're wired below with
  # plain `connect`/`feedback` instead of being chained into this.
  pid(:ctrl, kp: 8.0, ki: 4.0, kd: 0.2, filter_tf: 0.02) >> probe(:u_cmd)

  use lag_stage, as: :actuator, tau: 0.2
  use lag_stage, as: :sensor, tau: 0.05
  scope :out, title: "Cascaded-lag position control"

  connect :setpoint, to: {:err, 0}
  connect :err, to: :ctrl
  connect :u_cmd, to: :actuator
  connect :actuator, to: :sensor
  feedback from: :sensor, to: {:err, 1} # closes the loop; readability sugar for connect

  connect :actuator, to: :out, as: :actuator_out
  connect :sensor, to: :out, as: :measured, display: "Measured position"
  connect :setpoint, to: :out
end

result = model.run
puts "logged signals          : #{result.keys.join(", ")}"
puts "internal subsystem names : #{model.block_index.keys.select { |k| k.includes?('.') }}"
puts "final measured position  : #{result[:measured].last.round(4)} (target 1.0)"
puts "peak command             : #{result[:u_cmd].map(&.abs).max.round(2)}"

result.plot("cascaded_lags.html")
model.render("cascaded_lags_diagram.html")
model.report(result, "cascaded_lags_report.html") # diagram + sparklines + plots, one file
puts "written: cascaded_lags.html, cascaded_lags_diagram.html, cascaded_lags_report.html"
