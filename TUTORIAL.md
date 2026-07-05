# CrySim Tutorial — from zero to a working simulation

A graduated, chapter-by-chapter walkthrough. Each chapter builds on the last and ends
with a complete, runnable model — copy it into a `.cr` file, `require "crysim"` (or
`require "../src/crysim"` from inside this repo's `examples/` folder), and run it. Every
snippet in this tutorial has been run against the actual library while writing it.

If you want the terse reference instead of a narrative — block-by-block tables, "which
block do I want" decision guides — see [README.md](README.md). This document is the
"first afternoon with CrySim" path; the README is the thing you come back to afterward.

## Table of contents

1. [Your first simulation](#1-your-first-simulation)
2. [Choosing a solver method](#2-choosing-a-solver-method)
3. [Closing a feedback loop](#3-closing-a-feedback-loop)
4. [Watching signals: probes, scopes, and the diagram](#4-watching-signals-probes-scopes-and-the-diagram)
5. [State-space systems](#5-state-space-systems)
6. [Discrete time and multi-rate](#6-discrete-time-and-multi-rate)
7. [Nonlinearities and logic](#7-nonlinearities-and-logic)
8. [Reusable subsystems](#8-reusable-subsystems)
9. [Reports and diagnostics](#9-reports-and-diagnostics)
10. [The fast-path: when a model can skip the general engine](#10-the-fast-path-when-a-model-can-skip-the-general-engine)
11. [Writing your own block](#11-writing-your-own-block)
12. [Where to go next](#12-where-to-go-next)

---

## 1. Your first simulation

Every CrySim model is a block diagram, described with a small DSL, that produces a
`SimResult` when you run it. The three things every model needs: a duration and step
size for the solver, at least one block, and at least one wire connecting blocks
together.

Here's the smallest meaningful one: a step input through a first-order low-pass filter.

```crystal
require "crysim"

model = CrySim.model "first_order_filter" do
  duration 5.0   # simulate 5 seconds
  dt 0.01        # solver step: 10 ms

  step :input, amplitude: 1.0             # a step to 1.0 at t=0
  tf :filter, num: [1.0], den: [1.0, 1.0] # G(s) = 1 / (s + 1)
  scope :out                              # where logged signals end up

  connect :input, to: :filter
  connect :filter, to: :out, as: :output  # `as:` names the logged signal
end

result = model.run
puts result[:output].last   # => 0.9933 (five time constants in, close to steady state)
result.plot("chapter1.html") # open in a browser: an interactive Chart.js plot
```

A few things worth noticing already:

- `step`, `tf`, `scope` are **block declarations** — each takes a name (a `Symbol`,
  always the first argument) and block-specific keyword parameters.
- `connect src, to: dst` wires an output port to an input port. Without `{}`, both sides
  default to port 0 — the common case.
- `as: :output` **names** the signal for logging. Anything wired into a `scope` without
  an explicit `as:` still gets logged automatically, named after its source block — but
  naming it explicitly is clearer once a model has more than a couple of signals.
- `result[:output]` is an `Array(Float64)` — one value per solver step. `result.t` is the
  matching array of time values.

Run it, then open `chapter1.html` — you should see an S-curve climbing from 0 toward 1.0.

## 2. Choosing a solver method

Every model picks a numerical integration method with `method :rk4` (the default),
`:euler`, or `:midpoint`. RK4 is the right choice almost all the time — smooth,
continuous dynamics, which is most models. There's exactly one situation where it quietly
gives a wrong answer: **impulse response**.

```crystal
require "crysim"

tau = 2.0
model = CrySim.model "impulse_demo" do
  duration 1.0
  dt 0.01
  method :midpoint          # see why, below
  impulse :u, area: 1.0, time: 0.0
  tf :plant, num: [1.0], den: [tau, 1.0]
  scope :out
  connect :u, to: :plant
  connect :plant, to: :out, as: :y
end

result = model.run
puts result[:y][1].round(4) # close to the analytic h(dt) = (1/tau)*exp(-dt/tau)
```

Here's the concrete reason `:midpoint` is the right call for this, not `:rk4`: `impulse`
models a Dirac impulse as a rectangular pulse exactly one solver step wide. RK4
evaluates the derivative four times per step — at `t`, `t+dt/2` (twice), and `t+dt` — and
that *last* evaluation lands exactly on the pulse's excluded right edge. RK4 always
misses that one evaluation's contribution, so it converges to **5/6 of the correct
value** as `dt → 0` — not to the correct value. `:midpoint` only evaluates at `t` and
`t+dt/2`, never the edge, so it converges to the *correct* value as `dt` shrinks, and
it's more accurate than `:euler` at every `dt` along the way. Swap `method :midpoint` for
`method :rk4` in the snippet above and compare `result[:y][1]` at a few different `dt`
values to see the RK4 bias for yourself — it won't budge from ~5/6 no matter how small
`dt` gets.

The rule of thumb: **default to `:rk4`; switch to `:midpoint` specifically for `impulse`
response work**, or anything else with an input that's discontinuous exactly on the
solver's own step grid. See the README's ["Choosing a solver
method"](README.md#choosing-a-solver-method-euler-vs-midpoint-vs-rk4) for the full
comparison table.

## 3. Closing a feedback loop

Open-loop, a system settles wherever its own dynamics take it. To make it track a
*target* instead, close a loop: measure the output, compare it to a reference, and feed
the error into a controller.

```crystal
require "crysim"

model = CrySim.model "closed_loop" do
  duration 3.0
  dt 0.001

  # >> chains a straight-line path: each block's output feeds the next
  # one's input (port 0 to port 0). It's sugar for repeated `connect`.
  step(:setpoint, amplitude: 1.0) >>
    sum(:error, signs: "+-") >>
    pid(:controller, kp: 5.0, ki: 2.0) >>
    tf(:plant, num: [1.0], den: [1.0, 1.0])

  scope :out

  # The loop-closing wire is always explicit — chains only cover the
  # straight-line part. `feedback` is `connect` under a name that signals
  # intent; CrySim finds feedback edges structurally either way.
  feedback from: :plant, to: {:error, 1}

  connect :plant, to: :out, as: :y
  connect :setpoint, to: :out # also route the reference to the scope, to compare
end

result = model.run
puts result[:y].last # => 0.9578, tracking the setpoint of 1.0
result.plot("chapter2.html")
```

`>>` and `feedback` are pure sugar — you could write everything with `connect` instead,
and CrySim would build the exact same model. Reach for `>>` on the linear part of a
diagram, `connect` for anything with a specific port, a fan-out, or a label, and
`feedback` purely to make the loop-closing wire easy to spot when reading the model back
later — see the README's ["Wiring: `connect` vs. `>>` vs. `feedback`"](README.md) section
for the full decision guide.

## 4. Watching signals: probes, scopes, and the diagram

Two mechanisms get a signal into `SimResult` and they're easy to conflate at first:

- **`scope`** is a *terminal*: it has inputs but no output, and every signal wired into
  it gets logged and grouped into the same plot panel.
- **`probe`** is *inline*: 1 input, 1 output, pass-through (`y = u`). It taps a signal
  that keeps flowing to whatever comes after it — useful for observing something in the
  *middle* of a chain, like a controller's command on its way to the plant, without
  disrupting the wiring.

```crystal
model = CrySim.model "with_probe" do
  duration 3.0
  dt 0.001
  step(:setpoint, amplitude: 1.0) >> sum(:error, signs: "+-") >>
    pid(:controller, kp: 5.0, ki: 2.0) >> probe(:command) >>
    tf(:plant, num: [1.0], den: [1.0, 1.0])
  scope :out
  feedback from: :plant, to: {:error, 1}
  connect :plant, to: :out, as: :y
end

result = model.run
puts result.keys # => ["command", "y"] — the probe logged itself automatically
puts result[:command].max.round(2) # peak controller effort

model.render("chapter3_diagram.html") # SVG block diagram — no simulation needed
```

`model.render` doesn't require a completed (or even valid) run — it's a pure function of
the model's structure, so it's often the fastest way to sanity-check that a diagram
built with `connect`/`>>`/`feedback`/`use` actually wired up the way you intended. Open
`chapter3_diagram.html`: you'll see the feedback wire routed in its own lane below the
main diagram, and the `command` probe drawn as a small pass-through pill between the PID
and the plant.

## 5. State-space systems

`tf` is convenient when a system is naturally given as a transfer function. For
multi-input/multi-output systems, or when you already think in state-space, use `ss`
directly with A/B/C/D matrices.

```crystal
model = CrySim.model "mass_spring" do
  duration 5.0
  dt 0.001

  step :force, amplitude: 1.0
  # m*x'' + c*x' + k*x = F(t), state = [position, velocity], m=1, k=4, c=0.5
  ss :mechanism, a: [[0.0, 1.0], [-4.0, -0.5]], b: [[0.0], [1.0]],
                 c: [[1.0, 0.0]], d: [[0.0]],
                 state_names: [:position, :velocity] # auto-logs both states
  scope :out

  connect :force, to: :mechanism
  connect :mechanism, to: :out, as: :position
end

result = model.run
puts result[:position].last # the real output, wired to the scope
puts result[:velocity].last # auto-logged because it was named — no extra wiring needed

# Reach back into cryspace for analysis without leaving the model:
puts model.state_space_of(:mechanism).poles
```

`state_names:` is worth calling out: naming a state turns it into an automatically
logged signal (role `:state`) *without wiring it anywhere* — useful for inspecting
internal variables (like velocity here) that don't have their own physical output port.

## 6. Discrete time and multi-rate

Three tools cover discrete-time systems:

```crystal
model = CrySim.model "discrete_demo" do
  duration 2.0
  dt 0.01

  step :u, amplitude: 1.0
  tf :plant_c, num: [1.0], den: [0.5, 1.0]        # the continuous original
  discretize :plant_d, from: :plant_c, dt: 0.01   # a ZOH-discretized twin, no retyping
  scope :out

  connect :u, to: :plant_c
  connect :u, to: :plant_d
  connect :plant_c, to: :out, as: :continuous
  connect :plant_d, to: :out, as: :discrete
end

result = model.run
puts "continuous: #{result[:continuous].last.round(4)}, discrete: #{result[:discrete].last.round(4)}"
```

`discretize` reuses an already-declared block's dynamics (via cryspace's `sample`) —
handy for comparing a continuous design against its discretized implementation side by
side, as above. For a system that's inherently sampled, declare it directly with `dss`.

A `dss` (or a `pid` with `rate:`) can also run *slower* than the model's base `dt` — an
integer multiple of it, checked at build time:

```crystal
pid :outer_loop, kp: 1.0, rate: 5   # updates every 5 base steps, not every step
dss :slow_sensor, a: [[0.9]], b: [[1.0]], c: [[1.0]], d: [[0.0]], dt: 0.01 # 10x the 0.001 base dt
```

Both hold their output constant (zero-order hold) between updates, exactly like a real
sampled controller or sensor would.

## 7. Nonlinearities and logic

- `saturation`/`deadzone` — actuator limits, sensor dead bands.
- `switch` — pick between two signals: `criteria:`/`threshold:` for a plain relational
  test (the common case), or `condition:` (an `EEEval::CondParser` expression) for
  exact-value/mode gating — see the README for why those are two different mechanisms.
- `lookup_table` — a nonlinear characteristic curve from breakpoints/values, linearly
  interpolated (`extrapolate:` past the edges, or clamp — the default).
- `rate_limiter` — a signal can't move faster than a physical actuator would allow.

```crystal
model = CrySim.model "valve" do
  duration 0.1
  dt 0.01
  ramp :command, slope: 10.0
  lookup_table :valve, breakpoints: [0.0, 0.5, 1.0], values: [0.0, 90.0, 100.0]
  rate_limiter :actuator, rising_rate: 200.0, falling_rate: -400.0
  scope :out
  connect :command, to: :valve
  connect :valve, to: :actuator
  connect :actuator, to: :out, as: :flow
end
```

## 8. Reusable subsystems

When the same sub-diagram shows up more than once with different parameters — two motor
stages, two filter lags — a `subsystem` avoids repeating the wiring:

```crystal
motor = CrySim.subsystem("dc_motor") do |sub, params|
  # `sub` plays the role the model builder normally does implicitly — a
  # template body takes it explicitly, since Crystal can't rebind a
  # stored block's receiver the way Ruby's instance_eval does.
  sub.tf :dynamics, num: [params[:k]], den: [params[:tau], 1.0]
  sub.inport  :v_in,  to: :dynamics
  sub.outport :theta, from: :dynamics
end

model = CrySim.model "two_motors" do
  duration 3.0
  dt 0.001
  step :cmd, amplitude: 1.0
  use motor, as: :m1, k: 1.0, tau: 0.5
  use motor, as: :m2, k: 2.0, tau: 1.0
  scope :out
  connect :cmd, to: :m1
  connect :cmd, to: :m2
  connect :m1, to: :out, as: :theta1
  connect :m2, to: :out, as: :theta2
end

result = model.run
puts "m1: #{result[:theta1].last.round(4)}, m2: #{result[:theta2].last.round(4)}"
puts model.block_index.keys.select { |k| k.includes?('.') } # ["m1.dynamics", "m2.dynamics"]
```

Each `use` inlines a fresh copy of the template with its blocks prefixed by the instance
name; from the outside, the instance name itself (`:m1`) acts as a single input/output
port. v0.3's subsystems are SISO (exactly one inport, one outport per template) — see the
README's Roadmap for what's planned beyond that.

## 9. Reports and diagnostics

`model.render`/`result.plot` are useful separately; `report` combines them — the block
diagram, with a small sparkline of each wire's own signal next to its label, followed by
the same plot panels — in one file:

```crystal
model.report(result, "chapter8_report.html")
```

And when a model produces a NaN or Infinity — a division by zero in a custom `fn`
expression, an unstable pole, an out-of-range input — CrySim fails fast, naming exactly
where:

```crystal
broken = CrySim.model "broken" do
  duration 0.05
  dt 0.01
  ramp :u, slope: 1.0
  fn :risky, expr: "1/(u-0.02)" # a singularity partway through the run
  scope :out
  connect :u, to: :risky
  connect :risky, to: :out, as: :y
end

begin
  broken.run
rescue err : CrySim::NonFiniteValueError
  puts err.message # "block :risky produced Infinity at t=0.02 (output port 0) — ..."
  broken.render_error(err, "diagnosis.html") # the offending block, highlighted in red
end
```

## 10. The fast-path: when a model can skip the general engine

If a model happens to be *exactly* one source feeding a simple chain of continuous, SISO
`ss`/`dss`/`tf` blocks into one sink — no branching, no feedback, nothing else — CrySim
can flatten it into a single `CrySpace::StateSpace` and simulate it with cryspace's own
vectorized `simulate` instead of the general co-simulation engine:

```crystal
model = CrySim.model "fast_chain" do
  duration 2.0
  dt 0.001
  step :u, amplitude: 1.0
  tf :stage1, num: [1.0], den: [0.5, 1.0]
  tf :stage2, num: [2.0], den: [1.0, 1.0]
  scope :out
  connect :u, to: :stage1
  connect :stage1, to: :stage2
  connect :stage2, to: :out, as: :y
end

slow = model.run       # the general engine — always works
fast = model.run_fast  # the fast-path — only for this narrow shape

puts (slow[:y].last - fast[:y].last).abs # ~1e-15: floating-point identical
```

Most real models (anything with a `pid`, a `sum` closing a loop, more than one signal
path) don't qualify — `run_fast`/`to_state_space` will say exactly why not rather than
silently falling back to the general engine. Treat it as a speed option for that one
shape, not a replacement for `model.run`.

## 11. Writing your own block

When nothing built in fits, subclass `CrySim::Block` — the same interface every built-in
block implements — and register an instance with `block`:

```crystal
class CoulombFriction < CrySim::Block
  def initialize(name : Symbol, @mu : Float64)
    super(name.to_s, 1, 1) # 1 input (velocity), 1 output (friction force); Block's own identity is a String
  end

  def output(t : Float64, x : Array(Float64), u : Array(Float64), y : Array(Float64))
    y[0] = -@mu * Math.tanh(u[0] * 100.0) # smoothed sign(velocity), avoids a hard discontinuity
  end
end

model = CrySim.model "with_friction" do
  duration 3.0
  dt 0.001
  # ... other blocks ...
  block CoulombFriction.new(:friction, mu: 0.3)
  # ... wire :friction like any other block ...
end
```

`output` is required; a stateful block also implements `derivative` (continuous) or
`update_sample`/`commit_sample` (sampled, like a discrete state-space block) — see
`src/crysim/block.cr`'s doc comments for the full contract, and
`examples/05_custom_block.cr` for a complete, runnable version of this example.

## 12. Where to go next

- [README.md](README.md) — the reference: every built-in block, the "which one do I
  want" decision tables (sources, `probe` vs. `scope`, `connect` vs. `>>` vs. `feedback`,
  continuous dynamics, `switch` modes), installation, and the full roadmap.
- `examples/` — eight complete, runnable programs, each focused on a specific slice of
  the library (PID control, signal sources, state-space, saturation/anti-windup, custom
  blocks, and the v0.2/v0.3/robustness feature tours).
- `spec/crysim_spec.cr` — every behavior described in this tutorial and the README is
  backed by a spec, cross-checked in several places against analytic solutions and
  against cryspace's own `step_response`/`impulse_response`.
- `ANALISI_CRYSIM.md`, `PIANO_FEATURE.md`, `PIANO_STATESPACE.md` — the original design
  analysis and the feature backlog/roadmap, if you want the "why," not just the "how."
