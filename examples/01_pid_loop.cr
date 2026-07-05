require "../src/crysim"

# Closed-loop DC motor position control — the reference example from the
# design analysis (ANALISI_CRYSIM.md §4.2), with an inline probe on the
# control command.

model = CrySim.model "dc_motor_position" do
  duration 5.0
  dt 0.001
  method :rk4

  step :ref, amplitude: 1.0, start_time: 0.1
  sum :err, signs: "+-"
  pid :ctrl, kp: 12.0, ki: 3.0, kd: 0.8, u_min: -24.0, u_max: 24.0
  probe :u_mon
  tf :motor, num: [2.0], den: [0.5, 1.0, 0.0]
  gain :sensor, k: 1.0
  scope :out, title: "Risposta posizione"

  connect :ref, to: {:err, 0}
  connect :sensor, to: {:err, 1}
  connect :err, to: :ctrl
  connect :ctrl, to: :u_mon
  connect :u_mon, to: :motor
  connect :motor, to: :sensor
  # explicit display caption as metadata: shown on the SVG wire label and
  # as the plot legend entry, e.g. "Posizione motore (rad) (output)"
  connect :motor, to: :out, as: :position, display: "Posizione motore (rad)"
  connect :ref, to: :out # role auto-inferred as :input (Step is a source)
end

result = model.run

pos = result[:position]
puts "logged signals : #{result.keys.join(", ")}"
result.keys.each do |k|
  puts "  :#{k.to_s.ljust(10)} -> #{result.display_name(k)}"
end
puts "final position : #{pos.last.round(4)} (target 1.0)"
puts "peak position  : #{pos.max.round(4)} at t=#{result.t[pos.index(pos.max).not_nil!].round(3)}s"
puts "peak command   : #{result[:u_mon].max.round(2)} (saturation at 24.0)"

result.to_csv("dc_motor_run.csv")
result.plot("dc_motor_response.html")
model.render("dc_motor_diagram.html")
puts "written: dc_motor_run.csv, dc_motor_response.html, dc_motor_diagram.html"
