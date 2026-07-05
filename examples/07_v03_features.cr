require "../src/crysim"

# Tour of the v0.3 additions: the scoped LTI fast-path, the `discretize`
# sugar, `unit_delay`, `switch`, and multi-rate (a slower dss / a slower
# PID). Each section is self-contained.

# ---------------------------------------------------------------------------
# 1. Scoped LTI fast-path: a pure series chain (source -> ss/dss/tf -> ...
#    -> sink, no branching/feedback/non-LTI block) can skip the general
#    co-simulation engine entirely and run through cryspace's own
#    vectorized `simulate` on a single flattened StateSpace instead.
#    `model.run_fast` matches `model.run` to floating-point precision here
#    — same RK4, same dt, just without the per-block dispatch overhead.
fast_model = CrySim.model "three_stage_filter" do
  duration 3.0
  dt 0.001
  step :u, amplitude: 1.0
  tf :stage1, num: [1.0], den: [0.5, 1.0]
  ss :stage2, a: [[-2.0]], b: [[1.0]], c: [[1.0]], d: [[0.0]]
  tf :stage3, num: [3.0], den: [1.0, 1.0]
  scope :out
  connect :u, to: :stage1
  connect :stage1, to: :stage2
  connect :stage2, to: :stage3
  connect :stage3, to: :out, as: :y
end

slow_result = fast_model.run
fast_result = fast_model.run_fast
max_diff = slow_result[:y].each_with_index.map { |v, i| (v - fast_result[:y][i]).abs }.max
puts "-- LTI fast-path --"
puts "run vs. run_fast max diff : #{max_diff} (both RK4, same dt — should be ~machine epsilon)"
puts "flattened system poles    : #{fast_model.to_state_space.poles.map { |p| p.round(3) }}"
fast_model.render("three_stage_filter_diagram.html")

# ---------------------------------------------------------------------------
# 2. discretize: turn stage1 into a ZOH-discretized twin without retyping
#    its transfer function, and compare the two step responses.
discretize_model = CrySim.model "discretize_demo" do
  duration 2.0
  dt 0.01
  step :u, amplitude: 1.0
  tf :plant_c, num: [1.0], den: [0.5, 1.0]
  discretize :plant_d, from: :plant_c, dt: 0.01, method: :zoh
  scope :out
  connect :u, to: :plant_c
  connect :u, to: :plant_d
  connect :plant_c, to: :out, as: :continuous
  connect :plant_d, to: :out, as: :discrete
end
dr = discretize_model.run
puts
puts "-- discretize --"
puts "continuous final : #{dr[:continuous].last.round(4)}"
puts "discrete final   : #{dr[:discrete].last.round(4)} (ZOH at the same dt as the solver: matches closely)"

# ---------------------------------------------------------------------------
# 3. unit_delay: the discrete z^-1 primitive — output holds x0, then
#    trails the input by exactly one solver step.
delay_model = CrySim.model "unit_delay_demo" do
  duration 0.01
  dt 0.001
  ramp :u, slope: 1.0
  unit_delay :d, x0: 0.0
  scope :out
  connect :u, to: :d
  connect :d, to: :out, as: :delayed
  connect :u, to: :out
end
dl = delay_model.run
puts
puts "-- unit_delay --"
puts "u       : #{dl[:u].map { |v| v.round(3) }}"
puts "delayed : #{dl[:delayed].map { |v| v.round(3) }}"

# ---------------------------------------------------------------------------
# 4. switch: threshold mode (pure Crystal >, the common case) and
#    condition mode (EEEval::CondParser — exact-value/mode gating only,
#    since CondParser has no relational operators, just ==/!=/&&/||).
switch_model = CrySim.model "switch_demo" do
  duration 0.02
  dt 0.001
  constant :hot, value: 100.0
  constant :cold, value: 0.0
  ramp :sensor, slope: 1.0 # crosses the threshold partway through
  switch :thermostat, criteria: :greater_than, threshold: 0.01
  scope :out
  connect :hot, to: {:thermostat, 0}
  connect :sensor, to: {:thermostat, 1}
  connect :cold, to: {:thermostat, 2}
  connect :thermostat, to: :out, as: :heater
end
sw = switch_model.run
puts
puts "-- switch (threshold mode) --"
puts "heater output: #{sw[:heater].map(&.to_i)}"

# ---------------------------------------------------------------------------
# 5. multi-rate: a dss ticking 10x slower than the base solver step, and a
#    PID running its own control law at a slower, explicit rate.
multirate_model = CrySim.model "multirate_demo" do
  duration 0.05
  dt 0.001
  constant :u, value: 1.0
  dss :slow_filter, a: [[0.9]], b: [[1.0]], c: [[1.0]], d: [[0.0]], dt: 0.01 # 10x base dt
  ramp :ref, slope: 1.0
  pid :outer_loop, kp: 1.0, rate: 5 # updates every 5 base steps
  scope :out
  connect :u, to: :slow_filter
  connect :slow_filter, to: :out, as: :filtered
  connect :ref, to: :outer_loop
  connect :outer_loop, to: :out, as: :command
end
mr = multirate_model.run
filtered_changes = (1...mr[:filtered].size).count { |i| mr[:filtered][i] != mr[:filtered][i - 1] }
command_changes = (1...mr[:command].size).count { |i| mr[:command][i] != mr[:command][i - 1] }
puts
puts "-- multi-rate --"
puts "slow_filter (rate 10) value changes : #{filtered_changes} over #{mr[:filtered].size} samples"
puts "outer_loop PID (rate 5) value changes: #{command_changes} over #{mr[:command].size} samples"
