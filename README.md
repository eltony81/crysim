# CrySim

**A Simulink-inspired block-diagram simulation library for Crystal**, built on top of
[cryspace](https://github.com/eltony81/cryspace), [num.cr](https://github.com/eltony81/num.cr)
and [easy_expression_eval](https://github.com/eltony81/easy_expression_eval).

While cryspace provides the *analysis tools* (state-space, transfer functions, LQR, Kalman,
ODE solvers), CrySim provides the *composition*: a model described as a diagram of
interconnected blocks through a **DSL**, with the engine taking care of evaluation ordering,
numerical integration and result collection.

**New to CrySim?** [TUTORIAL.md](TUTORIAL.md) is a graduated, chapter-by-chapter
walkthrough from your first simulation to subsystems, multi-rate, and diagnostics. This
README is the reference to come back to afterward.

Design document: [ANALISI_CRYSIM.md](ANALISI_CRYSIM.md) (Italian).

## Quick example

```crystal
require "crysim"

model = CrySim.model "dc_motor_position" do
  duration 5.0
  dt 0.001
  method :rk4

  step  :ref,    amplitude: 1.0, start_time: 0.1
  sum   :err,    signs: "+-"
  pid   :ctrl,   kp: 12.0, ki: 3.0, kd: 0.8, u_min: -24.0, u_max: 24.0
  probe :u_mon                     # inline monitor on the command signal
  tf    :motor,  num: [2.0], den: [0.5, 1.0, 0.0]
  gain  :sensor, k: 1.0
  scope :out,    title: "Position response"

  connect :ref,    to: {:err, 0}
  connect :sensor, to: {:err, 1}   # feedback '-'
  connect :err,    to: :ctrl
  connect :ctrl,   to: :u_mon
  connect :u_mon,  to: :motor
  connect :motor,  to: :sensor
  connect :motor,  to: :out, as: :position
  connect :ref,    to: :out
end

result = model.run
puts result[:position].last          # logged signals by name
result.plot("response.html")         # interactive Chart.js plot
result.to_csv("run.csv")
model.render("diagram.html")         # SVG block diagram of the model
```

Compiled with `-Darrow` (num.cr's Apache Arrow backend), `SimResult` also gains
`to_feather(path)`/`to_parquet(path)` — the same `t` + logged-signal columns as `to_csv`,
but as a compact, typed, columnar file that pandas/polars/DuckDB open natively, for long
runs or many-column results where re-parsing CSV text gets expensive.

## Features

- **DSL** for declarative model composition (`CrySim.model do ... end`), with build-time
  validation: unconnected ports, unknown blocks, **algebraic loop detection** with
  didactic error messages.
- **`>>` chain sugar** for the straight-line case: `step(:r) >> sum(:e, signs: "+-") >>
  pid(:c, kp: 5.0) >> tf(:g, num: [1.0], den: [1.0, 1.0])` wires port 0 to port 0 down the
  chain. Fan-out, feedback, and non-zero ports still use `connect`.
- **`feedback from:/to:`** — sugar over `connect` that signals intent on a loop-closing
  wire. Purely readability: CrySim detects feedback edges structurally either way.
- **Subsystems** (`CrySim.subsystem` + `use`): a reusable, parametric block template,
  instantiated any number of times with its own prefixed blocks (`"m1.dynamics"`) and a
  single external in/out port usable as the instance name itself
  (`connect :ref, to: :m1`). See `examples/06_v02_features.cr`. v0.2 scope is SISO
  (exactly one inport/outport per template) — multi-port subsystems are deferred to v0.3,
  alongside Mux/Demux (see Roadmap).
- **Unified report** (`model.report(result, path)` / `result.report(model, path)`): the
  SVG block diagram — with a small sparkline of each wire's own logged signal drawn next
  to its label — followed by the same Chart.js plot panels as `result.plot`, in one file.
- **Simple multi-rate**: a `dss` runs at its own declared `dt` — an integer multiple of
  the base solver step (v0.3 requires a clean ratio; a non-integer one raises) — and `pid`
  accepts an explicit `rate:` multiplier for a slower outer loop. Both hold their output
  (ZOH) between updates, same as always.
- **`discretize`** — sugar that turns an already-declared continuous block (`ss`/`tf`)
  into a new `dss` twin via cryspace's `sample(dt, method:)`, without retyping its
  transfer function or matrices.
- **Scoped LTI fast-path** (`model.to_state_space`, `model.run_fast`): when a model is
  *exactly* one source feeding a simple chain of continuous, SISO `ss`/`dss`/`tf` blocks
  into one sink — no branching, no feedback, no other block type — CrySim flattens it to
  a single `CrySpace::StateSpace` via cryspace's `*` operator and simulates with
  cryspace's own vectorized `simulate` instead of the general engine. Matches `model.run`
  to floating-point precision on that narrow case; anything else raises with a specific
  message rather than guessing (see Roadmap for the general case).
- **NaN/Inf detection with context**: the first non-finite value produced during a run
  raises `CrySim::NonFiniteValueError` naming the exact block, port and simulation time
  that produced it, instead of surfacing many steps later as a mysteriously broken plot.
  `model.render_error(err, path)` highlights the culprit in red in the SVG diagram.
- **Fixed-step co-simulation engine** (Euler / RK4) reusing the block-diagram semantics of
  Simulink: topological evaluation order, stateful blocks break loops, sampled blocks
  (PID, Noise) held during solver substeps.
- **~25 built-in blocks**:
  - *Sources*: `constant`, `step`, `ramp`, `sine`, `cosine`, `impulse` (numerical Dirac),
    `pulse`, `sawtooth`, `triangle`, `chirp`, `noise` (seedable gaussian),
    `signal` (custom `Proc` **or** eeeval string expression).
  - *Math*: `gain`, `sum` (signed, N inputs), `product`, `saturation`, `deadzone`,
    `fn` (custom `Proc` or eeeval expression), `switch` (threshold `criteria:` or an
    eeeval `condition:` for exact-value gating — see "Choosing the right block"),
    `lookup_table` (1D piecewise-linear, `extrapolate:` or clamp past the edges).
  - *Continuous*: `integrator`, `unit_delay` (the discrete z⁻¹ primitive, always at the
    base rate), `rate_limiter` (slew-rate limit, asymmetric `rising_rate:`/`falling_rate:`),
    `tf` (via cryspace `TransferFunction#to_statespace`), `ss` (state-space
    matrices or an existing `CrySpace::StateSpace` via `sys:`, with optional
    `state_names:`/`output_names:` for auto-logged, readable states), `dss` (explicit
    discrete state-space, `x[k+1] = Ax[k] + Bu[k]`, own declared rate), `pid` (cryspace
    `PIDController`, anti-windup, optional `rate:`).
  - *Instrumentation*: `probe` — inline pass-through monitor, auto-logged, zero impact
    on results.
  - *Sinks*: `scope` (grouped plot panels).
- **SVG diagram renderer**: `model.to_svg` / `model.render(path)` — layered layout
  (sources left, sinks right), control-diagram glyphs (gain triangle, sum circle with
  signs, num/den fraction), feedback wires routed below the diagram, parameter tooltips.
- **Signal role & display metadata**: every logged signal is tagged `:input`, `:output`
  or `:monitor` (auto-inferred from the source block — sources are inputs, probes are
  monitors, everything else is an output; overridable with `role:`), and can carry an
  optional human-readable `display:` caption. Both are shown on the SVG wire label
  (color-coded) and in the plot legend, e.g. `connect :motor, to: :out, as: :position,
  display: "Posizione motore (rad)"` → *"Posizione motore (rad) (output)"*.
- **SimResult**: named signal logging, CSV export, Chart.js interactive plots
  (same template as cryspace), `tensor(key)` bridge to num.cr.
- **`Model#state_space_of(name)`**: extracts the `CrySpace::StateSpace` behind an `ss`/`dss`/`tf`
  block for direct analysis without leaving the model (`model.state_space_of(:plant).poles`,
  `.bode_plot(...)`, `.lqr(...)`). Detailed development plan in `PIANO_STATESPACE.md`.
- **eeeval expressions**: `signal :ref, expr: "0.5*t + 0.05*sin(2*pi*10*t)"` and
  `fn :sq, expr: "u^2"` — compiled once to AST, evaluated per step; serializable
  (foundation for the future YAML model loader).

## Choosing the right block

The block library has more than one way to do similar-looking things on purpose — each
option trades off differently. This section is the "which one do I actually want" guide.

### Sources — which waveform generator

| Use | When |
|---|---|
| `constant` | a fixed value with no time dependence — a bias, a disturbance level that never changes |
| `step` | an instantaneous change at `start_time` — classic step-response tests, or a setpoint that jumps |
| `ramp` | linear growth from `start_time` — track a *moving* target, or spin a plant up gently instead of shocking it with a step |
| `sine` / `cosine` | a single-frequency oscillation — frequency-response spot checks, periodic disturbances |
| `pulse` / `sawtooth` / `triangle` | periodic non-sinusoidal shapes — duty-cycle-dependent tests, PWM-like inputs |
| `impulse` | the numerical Dirac approximation (one solver step wide, area-normalized) — impulse response, same convention as cryspace's `impulse_response`. **⚠️ Don't use the default `method: :rk4` for impulse response work.** Its `k4` evaluation lands exactly on the pulse's excluded edge, so RK4 always undercounts the impulse's momentum by exactly 1/6 — *independent of `dt`* (verified: it converges to 5/6 of the analytic value, not to it, as `dt → 0`). Use `method: :midpoint` instead (see "Choosing a solver method" below) — it doesn't sample the edge, converges to the correct value as `dt → 0`, and is *more* accurate than `:euler` at every `dt` tested. See `Blocks::Impulse`'s doc comment for the full explanation. |
| `chirp` | a swept frequency — the natural input for system identification (cryspace's `ident` module) or a one-run frequency sweep |
| `noise` | seeded gaussian disturbance — disturbance rejection tests; fix `seed:` for reproducible Monte-Carlo-style runs |
| `signal` (`Proc` or `expr:`) | anything that doesn't fit the above — an arbitrary profile |

### Choosing a solver method: `:euler` vs. `:midpoint` vs. `:rk4`

`method :rk4` (the default) is the right choice for almost everything — smooth,
continuous dynamics, the common case. Two situations call for something else:

| Use | When |
|---|---|
| `:rk4` (default) | smooth continuous dynamics — the common case. 4th-order accurate, four derivative evaluations per step (at `t`, `t+dt/2` twice, and `t+dt`) |
| `:midpoint` | **use this for `impulse` response work**, or anything else with an input that's discontinuous exactly on the solver's own step grid. Only evaluates at `t` and `t+dt/2` — never at the right edge of a step — so it doesn't have RK4's blind spot for a pulse that's exactly one step wide. 2nd-order accurate: less precise than RK4 on smooth dynamics, but *more* precise than `:euler` at every `dt`, and — unlike RK4 on an impulse — its error shrinks as `dt` shrinks instead of sitting at a fixed, wrong fraction of the true value |
| `:euler` | rarely the best choice on its own merits (1st-order, `:midpoint` beats it even on the impulse case it used to be the fix for) — mainly useful as the simplest possible reference when debugging the solver itself |

The concrete finding behind that `:midpoint` recommendation: simulating a first-order
system's impulse response, the error vs. the analytic value shrinks roughly
proportionally to `dt` for `:midpoint` (10x smaller `dt` → ~10x smaller error), while
`:rk4`'s error converges to a *fixed* ~17% (5/6 of the correct value) no matter how small
`dt` gets — see `Blocks::Impulse`'s doc comment and the `method: :midpoint` spec
in `spec/crysim_spec.cr` for the full numbers.

### Probe vs. Scope — the one people mix up

Both end up logged in `SimResult`, but they exist for different reasons:

| | `probe` | `scope` |
|---|---|---|
| Ports | 1 in, 1 out — pass-through (`y = u`) | any number of inputs (grows per `connect`), **0 outputs** |
| Role in the diagram | sits **inline**: the signal keeps flowing to whatever comes after it | a **terminal**: nothing connects to a Scope, because it has no output |
| Effect on the simulation | none — adding/removing a probe never changes the numbers | none — same |
| What you get in the report | its own logged signal, plotted in the catch-all "Signals" panel | a **dedicated plot panel** shared by every signal wired into that same Scope |

**Rule of thumb**: use a `probe` when the signal's journey *continues* after being observed
(e.g. a controller's command on its way to the plant — see `:u_mon`/`:u_cmd` in the
examples). Use a `scope` when, for logging/plotting purposes, the signal's journey *ends*
there — even if, like a setpoint, it is *also* independently wired somewhere else in the
model (into the error sum, say). The practical payoff of picking `scope` correctly: wiring
both a setpoint and the measured output into the **same** Scope overlays them on one chart
so you can actually judge overshoot and tracking; routing them to two separate probes would
still log both, but scatter them into unrelated panels.

### Continuous dynamics — integrator vs. tf vs. ss vs. dss vs. pid

| Use | When |
|---|---|
| `integrator` | the single most primitive continuous stateful block (∫u dt) — hand-build custom dynamics from math blocks when nothing else fits |
| `unit_delay` | the discrete equivalent of `integrator` (z⁻¹, `y[k]=u[k-1]`), always at the base rate — hand-build discrete dynamics, or delay a signal by exactly one step |
| `rate_limiter` | a physical actuator can't move/react instantly — slew-rate-limit a reference or command, independent of anything else in the loop |
| `tf` | the dynamics are naturally given as num(s)/den(s) (a filter, a textbook plant) — converted once to state-space internally |
| `ss` | you already think in state-space, need MIMO, or want named states auto-logged (`state_names:`); also accepts an existing `CrySpace::StateSpace` via `sys:` (e.g. the result of `balred`, `to_observability_form`, or a discretization) |
| `dss` | the discrete twin of `ss` (`x[k+1] = Ax[k] + Bu[k]`) — an inherently sampled system, or (more often) reached via `discretize` on an existing continuous block rather than retyped by hand |
| `pid` | don't hand-assemble a PID from `sum` + `gain` + `integrator` — cryspace's `PIDController` already has a filtered derivative and clamping anti-windup; reach for math blocks only when the controller isn't a standard PID |

### `switch`: `criteria:`/`threshold:` vs. `condition:`

Two ways to state the selection test on a 3-port switch (data-if-true, control, data-if-false):

| Use | When |
|---|---|
| `criteria:`/`threshold:` | the common case — a plain relational test (`:greater_than`, `:less_equal`, ...) against a live signal, e.g. a thermostat or an actuator-limit gate |
| `condition:` (eeeval `CondParser`) | exact-value / discrete-mode gating (`"%.6f == 1.0"`) — `CondParser` only supports `==`/`!=`/`&&`/`\|\|` (its tokenizer doesn't recognize `>`/`<` at all), so it's for matching a mode flag, never a threshold |

### When to reach for `run_fast`

Only when a model happens to already be a single source → LTI chain → single sink with
nothing else in it (see `examples/07_v03_features.cr`) — a pure filter/plant cascade with
no controller, no branch, no feedback. `model.run` (the general engine) always works and
is the default; `run_fast` is purely a speed option for that one narrow shape, not a
replacement — most real models (anything with a `pid`, a `sum` closing a loop, or more
than one signal path) don't qualify, and `to_state_space`/`run_fast` will say exactly why
not rather than silently falling back.

### `signal`/`fn`: `Proc` vs. `expr:`

Both forms exist on every custom block. Default to the `Proc` form
(`->(t) { ... }` / `->(u, t) { ... }`) — it's type-checked by the compiler and faster at
run time. Reach for `expr:` (an eeeval string, e.g. `"0.5*t + 0.05*sin(2*pi*10*t)"`) only
when the formula needs to be **serializable** — loaded from a config file rather than
compiled into the program — since a `Proc` can't be written to disk and read back, but a
string can.

### Wiring: `connect` vs. `>>` vs. `feedback`

All three end up calling the same underlying wire creation — pick based on what the wire
*is*, not habit:

| Use | When |
|---|---|
| `>>` | a straight-line link between two real blocks, port 0 to port 0 — the common case in a signal chain |
| `connect` | anything `>>` can't express: a specific port (`{:err, 1}`), a fan-out (a source feeding two destinations), a labeled/`display:`ed signal, or either side being a subsystem instance (`:m1`) rather than a plain block |
| `feedback from:/to:` | exactly like `connect`, reserved for the wire that closes a loop — purely to make that wire easy to spot reading the model, since CrySim finds feedback edges structurally either way and renders them identically regardless of which method created them |

### Subsystems: when to reach for `use`

Building the same sub-diagram more than once with different parameters (two motor
stages, two filter lags, ...) — see `examples/06_v02_features.cr`. Not worth it for a
one-off block or a diagram you're not repeating; the inlining and prefixed naming add a
layer of indirection that only pays for itself on the second (and further) instance.

## Installation

```yaml
dependencies:
  crysim:
    github: eltony81/crysim
```

BLAS/LAPACK system libraries are required (through num.cr). See the cryspace README for
platform notes. `SimResult#to_feather`/`#to_parquet` additionally require compiling with
`-Darrow` (Apache Arrow GLib/Parquet-GLib system libraries, through num.cr's `arrow.cr`
dependency) — everything else builds and runs without it.

## Custom blocks

Subclass `CrySim::Block` (the S-function equivalent) and register it with `block`:

```crystal
class MyFriction < CrySim::Block
  def initialize(name : Symbol, @mu : Float64)
    super(name.to_s, 1, 1) # Block's identity is a String — see block.cr
  end

  def output(t, x, u, y)
    y[0] = @mu * Math.tanh(u[0] * 100.0)
  end
end

CrySim.model "with_friction" do
  # ...
  block MyFriction.new(:fric, 0.3)
end
```

## Examples & tests

```bash
crystal run examples/01_pid_loop.cr             # closed-loop PID with an inline probe
crystal run examples/02_signal_sources.cr       # sources tour + eeeval expressions
crystal run examples/03_mass_spring_damper.cr   # ss (A,B,C,D) vs. tf cross-check, state_names, state_space_of
crystal run examples/04_saturated_tracking.cr   # actuator saturation + sensor dead zone + anti-windup
crystal run examples/05_custom_block.cr         # extending CrySim with a custom CrySim::Block subclass
crystal run examples/06_v02_features.cr         # >> chain, feedback sugar, subsystems, unified report
crystal run examples/07_v03_features.cr         # LTI fast-path, discretize, unit_delay, switch, multi-rate
crystal run examples/08_piano_features.cr       # lookup_table, rate_limiter, NaN/Inf detection
crystal spec                                    # validated against analytic responses
```

## Roadmap

Shipped so far: v0.1 (core DSL, engine, ~25 blocks), v0.2 (`>>`/`feedback` sugar, SISO
subsystems, unified report), v0.3 (multi-rate, `unit_delay`, `discretize`, `switch`,
scoped LTI fast-path), and a robustness/block-library round (NaN/Inf detection, cryspace
equivalence testing, `lookup_table`, `rate_limiter`, CI, `method: :midpoint`). Planned
next: Mux/Demux (vector signals), multi-port subsystems, general LTI flattening, observer
blocks, adaptive RK45, an algebraic loop solver, a YAML model loader.

The detailed backlog — every planned feature by area with priority and rationale, what
shipped in which round and why, what's still deferred and what it's blocked on — is
tracked in one place: **[PIANO_FEATURE.md](PIANO_FEATURE.md)** (Italian).

## License

MIT
