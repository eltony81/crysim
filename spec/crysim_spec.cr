require "./spec_helper"

# Geometric helpers used by the "SVG diagram layout" spec below: parse the
# rendered SVG's own coordinates (blocks, wire path segments, label halos)
# and check for overlaps, rather than re-deriving layout independently —
# this catches renderer regressions regardless of which internal mechanism
# caused them.
module LayoutCheck
  record GBox, x1 : Float64, y1 : Float64, x2 : Float64, y2 : Float64

  def self.parse_blocks(svg : String) : Array(GBox)
    boxes = [] of GBox
    svg.scan(/<rect class="blk"[^>]*x="([\d.]+)"[^>]*y="([\d.]+)"[^>]*width="([\d.]+)"[^>]*height="([\d.]+)"/) do |m|
      x, y, w, h = m[1].to_f, m[2].to_f, m[3].to_f, m[4].to_f
      boxes << GBox.new(x, y, x + w, y + h)
    end
    svg.scan(/<circle class="blk"[^>]*cx="([\d.]+)"[^>]*cy="([\d.]+)"[^>]*r="([\d.]+)"/) do |m|
      cx, cy, rr = m[1].to_f, m[2].to_f, m[3].to_f
      boxes << GBox.new(cx - rr, cy - rr, cx + rr, cy + rr)
    end
    svg.scan(/<polygon class="blk" points="([^"]+)"/) do |m|
      pts = m[1].split.map { |p| p.split(",").map(&.to_f) }
      xs = pts.map(&.[0])
      ys = pts.map(&.[1])
      boxes << GBox.new(xs.min, ys.min, xs.max, ys.max)
    end
    boxes
  end

  def self.parse_halos(svg : String) : Array(GBox)
    svg.scan(/<rect class="halo" x="(-?[\d.]+)" y="(-?[\d.]+)" width="([\d.]+)" height="([\d.]+)"/).map do |m|
      x, y, w, h = m[1].to_f, m[2].to_f, m[3].to_f, m[4].to_f
      GBox.new(x, y, x + w, y + h)
    end
  end

  def self.parse_wire_segments(svg : String) : Array(Tuple(Float64, Float64, Float64, Float64))
    segs = [] of Tuple(Float64, Float64, Float64, Float64)
    svg.scan(/<path class="wire[^"]*" d="([^"]+)"/) do |m|
      cx = cy = 0.0
      m[1].scan(/([MHV])(-?[\d.]+)(?:,(-?[\d.]+))?/) do |tok|
        case tok[1]
        when "M"
          cx, cy = tok[2].to_f, tok[3].to_f
        when "H"
          nx = tok[2].to_f
          segs << {cx, cy, nx, cy}
          cx = nx
        when "V"
          ny = tok[2].to_f
          segs << {cx, cy, cx, ny}
          cy = ny
        end
      end
    end
    segs
  end

  def self.boxes_overlap?(a : GBox, b : GBox, shrink : Float64 = 0.5) : Bool
    ax1, ay1, ax2, ay2 = a.x1 + shrink, a.y1 + shrink, a.x2 - shrink, a.y2 - shrink
    !(ax2 <= b.x1 || ax1 >= b.x2 || ay2 <= b.y1 || ay1 >= b.y2)
  end

  def self.segment_crosses_box?(seg : Tuple(Float64, Float64, Float64, Float64), box : GBox, shrink : Float64 = 3.0) : Bool
    x1, y1, x2, y2 = seg
    bx1, by1, bx2, by2 = box.x1 + shrink, box.y1 + shrink, box.x2 - shrink, box.y2 - shrink
    return false if bx2 <= bx1 || by2 <= by1
    if y1 == y2
      return false unless by1 < y1 < by2
      lo, hi = {x1, x2}.min, {x1, x2}.max
      !(hi <= bx1 || lo >= bx2)
    else
      return false unless bx1 < x1 < bx2
      lo, hi = {y1, y2}.min, {y1, y2}.max
      !(hi <= by1 || lo >= by2)
    end
  end

  def self.parse_polyline_boxes(svg : String) : Array(GBox)
    svg.scan(/<polyline points="([^"]+)"/).map do |m|
      pts = m[1].split.map { |p| p.split(",").map(&.to_f) }
      xs = pts.map(&.[0])
      ys = pts.map(&.[1])
      GBox.new(xs.min, ys.min, xs.max, ys.max)
    end
  end

  def self.assert_no_overlaps(model : CrySim::Model, result : CrySim::SimResult? = nil)
    svg = result ? CrySim::Diagram::SvgRenderer.new(model, result).to_svg : model.to_svg
    blocks = parse_blocks(svg)
    halos = parse_halos(svg)
    segs = parse_wire_segments(svg)
    sparklines = parse_polyline_boxes(svg)

    segs.each do |seg|
      blocks.each do |box|
        segment_crosses_box?(seg, box).should be_false, "wire segment #{seg} crosses block #{box}"
      end
    end
    halos.each_with_index do |a, i|
      halos.each_with_index do |b, j|
        next unless j > i
        boxes_overlap?(a, b).should be_false, "label #{a} overlaps label #{b}"
      end
      blocks.each do |box|
        boxes_overlap?(a, box).should be_false, "label #{a} overlaps block #{box}"
      end
    end
    sparklines.each do |sp|
      blocks.each do |box|
        boxes_overlap?(sp, box).should be_false, "sparkline #{sp} overlaps block #{box}"
      end
    end
  end
end

describe CrySim do
  it "matches the analytic step response of a first-order system" do
    # G(s) = 1/(tau*s + 1), step at t=0: y(t) = 1 - e^(-t/tau)
    tau = 0.5
    model = CrySim.model "first_order" do
      duration 2.0
      dt 0.001
      step :u, amplitude: 1.0
      tf :plant, num: [1.0], den: [tau, 1.0]
      scope :out
      connect :u, to: :plant
      connect :plant, to: :out, as: :y
    end

    result = model.run
    y = result[:y]
    result.t.each_with_index do |t, i|
      expected = 1.0 - Math.exp(-t / tau)
      y[i].should be_close(expected, 1e-5)
    end
  end

  it "integrates a constant into a ramp" do
    model = CrySim.model "int" do
      duration 1.0
      dt 0.01
      constant :c, value: 3.0
      integrator :i, x0: 1.0
      scope :out
      connect :c, to: :i
      connect :i, to: :out, as: :x
    end

    result = model.run
    result[:x].first.should be_close(1.0, 1e-12)
    result[:x].last.should be_close(4.0, 1e-9)
  end

  it "evaluates math blocks through a chain" do
    model = CrySim.model "math" do
      duration 0.1
      dt 0.01
      constant :a, value: 2.0
      constant :b, value: 5.0
      sum :s, signs: "+-"
      gain :g, k: 10.0
      saturation :sat, min: -20.0, max: 20.0
      scope :out
      connect :a, to: {:s, 0}
      connect :b, to: {:s, 1}
      connect :s, to: :g
      connect :g, to: :sat
      connect :sat, to: :out, as: :y
    end

    # (2 - 5) * 10 = -30, saturated to -20
    model.run[:y].last.should be_close(-20.0, 1e-12)
  end

  it "gives identical results for Proc and eeeval expression sources" do
    build = ->(use_expr : Bool) do
      CrySim.model "src_#{use_expr}" do
        duration 0.5
        dt 0.001
        if use_expr
          signal :src, expr: "0.5*t + 0.05*sin(2*pi*10*t)"
        else
          signal :src, ->(t : Float64) { 0.5 * t + 0.05 * Math.sin(2.0 * Math::PI * 10.0 * t) }
        end
        integrator :i
        scope :out
        connect :src, to: :i
        connect :i, to: :out, as: :x
        connect :src, to: :out, as: :u
      end
    end

    r_proc = build.call(false).run
    r_expr = build.call(true).run
    r_proc[:x].each_with_index do |v, i|
      r_expr[:x][i].should be_close(v, 1e-12)
    end
  end

  it "supports eeeval expression Fn blocks" do
    model = CrySim.model "expr_fn" do
      duration 0.1
      dt 0.01
      constant :c, value: 3.0
      fn :sq, expr: "u^2 + 1"
      scope :out
      connect :c, to: :sq
      connect :sq, to: :out, as: :y
    end

    model.run[:y].last.should be_close(10.0, 1e-12)
  end

  it "probes do not alter the simulation and are auto-logged" do
    base = CrySim.model "no_probe" do
      duration 1.0
      dt 0.001
      step :u
      tf :plant, num: [1.0], den: [0.2, 1.0]
      scope :out
      connect :u, to: :plant
      connect :plant, to: :out, as: :y
    end

    probed = CrySim.model "with_probe" do
      duration 1.0
      dt 0.001
      step :u
      probe :mon
      tf :plant, num: [1.0], den: [0.2, 1.0]
      scope :out
      connect :u, to: :mon
      connect :mon, to: :plant
      connect :plant, to: :out, as: :y
    end

    r1 = base.run
    r2 = probed.run
    r2.keys.should contain("mon")
    r1[:y].each_with_index { |v, i| r2[:y][i].should be_close(v, 1e-12) }
  end

  it "detects algebraic loops at build time" do
    expect_raises(CrySim::AlgebraicLoopError, /algebraic loop/) do
      CrySim.model "loop" do
        duration 1.0
        dt 0.01
        gain :a, k: 2.0
        gain :b, k: 0.5
        connect :a, to: :b
        connect :b, to: :a
      end
    end
  end

  it "rejects unconnected input ports with a didactic error" do
    expect_raises(CrySim::ModelError, /input 1 of :err is not connected/) do
      CrySim.model "dangling" do
        duration 1.0
        dt 0.01
        step :u
        sum :err, signs: "+-"
        connect :u, to: {:err, 0}
      end
    end
  end

  it "simulates a closed loop with feedback wire" do
    # unity feedback around G(s) = 4/s  ->  closed loop 4/(s+4)
    model = CrySim.model "closed_loop" do
      duration 2.0
      dt 0.001
      step :ref
      sum :err, signs: "+-"
      tf :plant, num: [4.0], den: [1.0, 0.0]
      scope :out
      connect :ref, to: {:err, 0}
      connect :plant, to: {:err, 1}
      connect :err, to: :plant
      connect :plant, to: :out, as: :y
    end

    result = model.run
    y = result[:y]
    result.t.each_with_index do |t, i|
      y[i].should be_close(1.0 - Math.exp(-4.0 * t), 1e-5)
    end
    model.feedback_wires.size.should eq(1)
  end

  it "infers input/output/monitor roles and honors display captions" do
    model = CrySim.model "roles" do
      duration 0.1
      dt 0.01
      step :ref, amplitude: 1.0
      probe :mon
      gain :g, k: 2.0
      scope :out
      connect :ref, to: :mon
      connect :mon, to: :g
      connect :g, to: :out, as: :y, display: "Uscita amplificata"
      connect :ref, to: :out
    end

    result = model.run
    result.meta["ref"].role.should eq(:input)
    result.meta["mon"].role.should eq(:monitor)
    result.meta["y"].role.should eq(:output)
    result.meta["y"].display.should eq("Uscita amplificata")
    result.display_name(:y).should eq("Uscita amplificata (output)")
    result.display_name(:ref).should eq("ref (input)")

    svg = model.to_svg
    svg.should contain("Uscita amplificata (output)")
    svg.should contain("ref (input)")
    svg.should contain("sig-input")
    svg.should contain("sig-output")
  end

  it "lets an explicit role: override the automatic inference" do
    model = CrySim.model "role_override" do
      duration 0.1
      dt 0.01
      constant :c, value: 1.0
      scope :out
      connect :c, to: :out, as: :y, role: :state
    end

    model.run.meta["y"].role.should eq(:state)
  end

  it "ss with literal matrices matches tf for the same SISO dynamics" do
    # G(s) = 1/(tau*s + 1) in control-canonical form: A=[-1/tau], B=[1],
    # C=[1/tau], D=[0] (same construction as TransferFunction#to_statespace).
    tau = 0.5
    model_tf = CrySim.model "via_tf" do
      duration 2.0
      dt 0.001
      step :u, amplitude: 1.0
      tf :plant, num: [1.0], den: [tau, 1.0]
      scope :out
      connect :u, to: :plant
      connect :plant, to: :out, as: :y
    end

    model_ss = CrySim.model "via_ss" do
      duration 2.0
      dt 0.001
      step :u, amplitude: 1.0
      ss :plant, a: [[-1.0 / tau]], b: [[1.0]], c: [[1.0 / tau]], d: [[0.0]]
      scope :out
      connect :u, to: :plant
      connect :plant, to: :out, as: :y
    end

    y_tf = model_tf.run[:y]
    y_ss = model_ss.run[:y]
    y_tf.each_with_index { |v, i| y_ss[i].should be_close(v, 1e-9) }
  end

  it "rejects a discrete CrySpace::StateSpace passed to a continuous block" do
    sys = CrySpace::StateSpace.new(
      [[0.9]].to_tensor, [[1.0]].to_tensor, [[1.0]].to_tensor, [[0.0]].to_tensor, 0.01)
    expect_raises(CrySim::ModelError, /sistema discreto/) do
      CrySim::Blocks::StateSpaceBlock.new("bad", sys)
    end
  end

  it "ss sys: reuses a cryspace-transformed StateSpace directly" do
    tau = 0.5
    base = CrySpace::TransferFunction.new([1.0].to_tensor, [tau, 1.0].to_tensor).to_statespace
    transformed = base.to_observability_form

    model = CrySim.model "reused_sys" do
      duration 2.0
      dt 0.001
      step :u, amplitude: 1.0
      ss :plant, sys: transformed
      scope :out
      connect :u, to: :plant
      connect :plant, to: :out, as: :y
    end

    result = model.run
    y = result[:y]
    result.t.each_with_index do |t, i|
      y[i].should be_close(1.0 - Math.exp(-t / tau), 1e-5)
    end
  end

  it "dss matches the closed-form recurrence of a first-order discrete filter" do
    # x[k+1] = 0.9*x[k] + u[k], y[k] = x[k]  =>  x[k] = 10*(1 - 0.9^k) for u[k]=1
    model = CrySim.model "discrete_filter" do
      duration 0.05
      dt 0.01
      constant :u, value: 1.0
      dss :plant, a: [[0.9]], b: [[1.0]], c: [[1.0]], d: [[0.0]], dt: 0.01
      scope :out
      connect :u, to: :plant
      connect :plant, to: :out, as: :y
    end

    y = model.run[:y]
    y.each_with_index do |v, k|
      v.should be_close(10.0 * (1.0 - 0.9 ** k), 1e-9)
    end
  end

  it "ss sys: auto-dispatches to the discrete block matching an explicit dss" do
    sys = CrySpace::StateSpace.new(
      [[0.9]].to_tensor, [[1.0]].to_tensor, [[1.0]].to_tensor, [[0.0]].to_tensor, 0.01)

    model_auto = CrySim.model "auto" do
      duration 0.05
      dt 0.01
      constant :u, value: 1.0
      ss :plant, sys: sys
      scope :out
      connect :u, to: :plant
      connect :plant, to: :out, as: :y
    end

    model_explicit = CrySim.model "explicit" do
      duration 0.05
      dt 0.01
      constant :u, value: 1.0
      dss :plant, a: [[0.9]], b: [[1.0]], c: [[1.0]], d: [[0.0]], dt: 0.01
      scope :out
      connect :u, to: :plant
      connect :plant, to: :out, as: :y
    end

    y_auto = model_auto.run[:y]
    y_explicit = model_explicit.run[:y]
    y_auto.each_with_index { |v, i| y_explicit[i].should be_close(v, 1e-12) }
  end

  it "state_space_of extracts the underlying CrySpace::StateSpace for direct analysis" do
    tau = 0.5
    model = CrySim.model "extract" do
      duration 0.1
      dt 0.01
      step :u
      tf :plant, num: [1.0], den: [tau, 1.0]
      gain :g, k: 2.0
      scope :out
      connect :u, to: :plant
      connect :plant, to: :g
      connect :g, to: :out, as: :y
    end

    direct = CrySpace::TransferFunction.new([1.0].to_tensor, [tau, 1.0].to_tensor).to_statespace
    extracted = model.state_space_of(:plant)
    extracted.poles.should eq(direct.poles)

    expect_raises(CrySim::ModelError, /no state-space representation/) do
      model.state_space_of(:g)
    end
    expect_raises(CrySim::ModelError, /unknown block/) do
      model.state_space_of(:nope)
    end
  end

  it "state_names on a continuous ss block are auto-logged and shown in the tooltip" do
    # mass-spring-damper: m=1, k=20, c=1, states [pos, vel], observe pos
    model = CrySim.model "msd" do
      duration 1.0
      dt 0.001
      step :f, amplitude: 1.0
      ss :plant, a: [[0.0, 1.0], [-20.0, -1.0]], b: [[0.0], [1.0]],
                 c: [[1.0, 0.0]], d: [[0.0]],
                 state_names: [:pos, :vel], output_names: [:pos]
      scope :out
      connect :f, to: :plant
      connect :plant, to: :out, as: :position
    end

    result = model.run
    result.keys.should contain("pos")
    result.keys.should contain("vel")
    result.meta["pos"].role.should eq(:state)
    # vel is the time-derivative of pos (within RK4/finite-difference tolerance)
    dt = 0.001
    mid = result.t.size // 2
    approx_vel = (result[:pos][mid + 1] - result[:pos][mid - 1]) / (2 * dt)
    result[:vel][mid].should be_close(approx_vel, 1e-3)

    svg = model.to_svg
    svg.should contain("pos")
    svg.should contain("vel")
  end

  it "state_names on a dss block are auto-logged as :state" do
    model = CrySim.model "dss_named" do
      duration 0.05
      dt 0.01
      constant :u, value: 1.0
      dss :plant, a: [[0.9]], b: [[1.0]], c: [[1.0]], d: [[0.0]], dt: 0.01, state_names: [:x1]
      scope :out
      connect :u, to: :plant
      connect :plant, to: :out, as: :y
    end

    result = model.run
    result.keys.should contain("x1")
    result.meta["x1"].role.should eq(:state)
    result[:x1].should eq(result[:y]) # C=1, D=0: the named state equals the real output
  end

  it "renders an SVG diagram for a valid model" do
    model = CrySim.model "diagram" do
      duration 1.0
      dt 0.01
      step :u
      gain :g, k: 2.0
      scope :out
      connect :u, to: :g
      connect :g, to: :out, as: :y
    end

    svg = model.to_svg
    svg.should contain("<svg")
    svg.should contain(":u")
    svg.should contain("polygon") # gain triangle + arrows
  end

  describe "SVG diagram layout" do
    it "has no wire-through-block or label overlaps for a fan-out + feedback + long captions model" do
      model = CrySim.model "layout_check" do
        duration 1.0
        dt 0.01
        step :ref, amplitude: 1.0
        sum :err, signs: "+-"
        pid :ctrl, kp: 5.0, ki: 1.0, kd: 0.2
        probe :u_mon
        tf :plant, num: [1.0], den: [1.0, 1.0]
        gain :sensor, k: 1.0
        scope :out, title: "Layout check"

        connect :ref, to: {:err, 0}
        connect :sensor, to: {:err, 1}
        connect :err, to: :ctrl
        connect :ctrl, to: :u_mon
        connect :u_mon, to: :plant
        connect :plant, to: :sensor
        connect :plant, to: :out, as: :position, display: "A very long descriptive caption (rad)"
        connect :ref, to: :out
      end
      LayoutCheck.assert_no_overlaps(model)
    end

    it "has no overlaps for a multi-output ss block with named states" do
      model = CrySim.model "layout_check_states" do
        duration 1.0
        dt 0.01
        step :f, amplitude: 1.0
        ss :plant, a: [[0.0, 1.0], [-20.0, -1.0]], b: [[0.0], [1.0]],
                   c: [[1.0, 0.0]], d: [[0.0]], state_names: [:position, :velocity]
        scope :out
        connect :f, to: :plant
        connect :plant, to: :out, as: :position
      end
      LayoutCheck.assert_no_overlaps(model)
    end
  end

  describe "v0.2: chain sugar, feedback sugar, subsystems" do
    it ">> chain sugar wires identically to explicit connect" do
      tau = 0.5
      via_chain = CrySim.model "via_chain" do
        duration 2.0
        dt 0.001
        step(:u, amplitude: 1.0) >> tf(:plant, num: [1.0], den: [tau, 1.0]) >> scope(:out)
        connect :plant, to: :out, as: :y
      end
      via_connect = CrySim.model "via_connect" do
        duration 2.0
        dt 0.001
        step :u, amplitude: 1.0
        tf :plant, num: [1.0], den: [tau, 1.0]
        scope :out
        connect :u, to: :plant
        connect :plant, to: :out, as: :y
      end

      r1 = via_chain.run[:y]
      r2 = via_connect.run[:y]
      r1.each_with_index { |v, i| r2[i].should be_close(v, 1e-12) }
    end

    it "feedback from:/to: is equivalent to connect and is still detected as a back-edge" do
      model = CrySim.model "fb_sugar" do
        duration 2.0
        dt 0.001
        step :ref
        sum :err, signs: "+-"
        tf :plant, num: [4.0], den: [1.0, 0.0]
        scope :out
        connect :ref, to: {:err, 0}
        feedback from: :plant, to: {:err, 1}
        connect :err, to: :plant
        connect :plant, to: :out, as: :y
      end

      result = model.run
      y = result[:y]
      result.t.each_with_index do |t, i|
        y[i].should be_close(1.0 - Math.exp(-4.0 * t), 1e-5)
      end
      model.feedback_wires.size.should eq(1)
    end

    it "instantiates a parametric subsystem twice with independent parameters" do
      plant_stage = CrySim.subsystem("first_order_plant") do |sub, params|
        sub.tf :dynamics, num: [params[:k]], den: [params[:tau], 1.0]
        sub.inport :v_in, to: :dynamics
        sub.outport :y_out, from: :dynamics
      end

      model = CrySim.model "two_plants" do
        duration 2.0
        dt 0.001
        step :ref, amplitude: 1.0
        use plant_stage, as: :p1, k: 1.0, tau: 0.5
        use plant_stage, as: :p2, k: 2.0, tau: 1.0
        scope :out
        connect :ref, to: :p1
        connect :ref, to: :p2
        connect :p1, to: :out, as: :y1
        connect :p2, to: :out, as: :y2
      end

      result = model.run
      direct1 = CrySim.model("d1") do
        duration 2.0
        dt 0.001
        step :u, amplitude: 1.0
        tf :g, num: [1.0], den: [0.5, 1.0]
        scope :o
        connect :u, to: :g
        connect :g, to: :o, as: :y
      end.run
      direct2 = CrySim.model("d2") do
        duration 2.0
        dt 0.001
        step :u, amplitude: 1.0
        tf :g, num: [2.0], den: [1.0, 1.0]
        scope :o
        connect :u, to: :g
        connect :g, to: :o, as: :y
      end.run

      result[:y1].each_with_index { |v, i| direct1[:y][i].should be_close(v, 1e-12) }
      result[:y2].each_with_index { |v, i| direct2[:y][i].should be_close(v, 1e-12) }
      model.block_index.keys.should contain("p1.dynamics")
      model.block_index.keys.should contain("p2.dynamics")
    end

    it "rejects two subsystem instances with the same name" do
      stage = CrySim.subsystem("s") do |sub, _params|
        sub.gain :g, k: 1.0
        sub.inport :i, to: :g
        sub.outport :o, from: :g
      end

      expect_raises(CrySim::ModelError, /already used/) do
        CrySim.model "dup" do
          duration 0.1
          dt 0.01
          use stage, as: :p1, k: 1.0
          use stage, as: :p1, k: 1.0
        end
      end
    end

    it "rejects a subsystem template that declares no inport/outport" do
      stage = CrySim.subsystem("no_ports") do |sub, _params|
        sub.gain :g, k: 1.0
      end

      expect_raises(CrySim::ModelError, /must declare exactly one/) do
        CrySim.model "bad_sub" do
          duration 0.1
          dt 0.01
          use stage, as: :p1
        end
      end
    end
  end

  describe "v0.2: unified report with sparklines" do
    it "combines the diagram (with sparklines) and the plot panels, with no overlaps" do
      model = CrySim.model "report_check" do
        duration 1.0
        dt 0.001
        step :ref, amplitude: 1.0, start_time: 0.1
        sum :err, signs: "+-"
        pid :ctrl, kp: 5.0, ki: 2.0, kd: 0.3
        probe :u_mon
        tf :plant, num: [1.0], den: [0.5, 1.0]
        scope :out
        connect :ref, to: {:err, 0}
        connect :plant, to: {:err, 1}
        connect :err, to: :ctrl
        connect :ctrl, to: :u_mon
        connect :u_mon, to: :plant
        connect :plant, to: :out, as: :position
        connect :ref, to: :out
      end

      result = model.run
      svg = CrySim::Diagram::SvgRenderer.new(model, result).to_svg
      svg.should contain("<polyline")
      LayoutCheck.assert_no_overlaps(model, result)
    end

    it "plain to_svg (no result attached) draws no sparklines" do
      model = CrySim.model "no_sparkline" do
        duration 0.1
        dt 0.01
        step :u
        gain :g, k: 2.0
        scope :out
        connect :u, to: :g
        connect :g, to: :out, as: :y
      end

      model.to_svg.should_not contain("<polyline")
    end
  end

  describe "v0.3: simple multi-rate" do
    it "a dss slower than the base dt only updates every rate_steps-th step" do
      model = CrySim.model "multirate" do
        duration 0.1
        dt 0.001
        constant :u, value: 1.0
        dss :plant, a: [[0.9]], b: [[1.0]], c: [[1.0]], d: [[0.0]], dt: 0.01
        scope :out
        connect :u, to: :plant
        connect :plant, to: :out, as: :y
      end

      y = model.run[:y]
      changes = (1...y.size).count { |i| y[i] != y[i - 1] }
      changes.should eq(10) # 100 steps at rate 10 -> 10 updates
    end

    it "rejects a dss dt that isn't an integer multiple of the base dt" do
      expect_raises(CrySim::ModelError, /multiplo intero/) do
        CrySim.model "bad_rate" do
          duration 0.1
          dt 0.001
          constant :u, value: 1.0
          dss :plant, a: [[0.9]], b: [[1.0]], c: [[1.0]], d: [[0.0]], dt: 0.0015
          scope :out
          connect :u, to: :plant
          connect :plant, to: :out, as: :y
        end.run
      end
    end

    it "pid rate: runs the controller at a slower, explicit rate" do
      model = CrySim.model "pid_rate" do
        duration 0.05
        dt 0.001
        ramp :ref, slope: 1.0
        pid :ctrl, kp: 1.0, ki: 0.0, kd: 0.0, rate: 5
        scope :out
        connect :ref, to: :ctrl
        connect :ctrl, to: :out, as: :u
      end

      u = model.run[:u]
      changes = (1...u.size).count { |i| u[i] != u[i - 1] }
      changes.should eq(10) # 50 steps at rate 5 -> 10 updates
    end
  end

  describe "v0.3: unit_delay" do
    it "holds x0 for one step, then outputs the previous input" do
      model = CrySim.model "delay_test" do
        duration 0.01
        dt 0.001
        ramp :u, slope: 1.0
        unit_delay :d, x0: -1.0
        scope :out
        connect :u, to: :d
        connect :d, to: :out, as: :y
        connect :u, to: :out
      end

      result = model.run
      u = result[:u]
      y = result[:y]
      y[0].should eq(-1.0)
      (1...u.size).each { |i| y[i].should be_close(u[i - 1], 1e-12) }
    end
  end

  describe "v0.3: switch (threshold and eeeval-condition modes)" do
    it "criteria:/threshold: selects between the two data inputs" do
      model = CrySim.model "switch_threshold" do
        duration 0.02
        dt 0.001
        constant :a, value: 10.0
        constant :b, value: -10.0
        ramp :ctrl, slope: 1.0
        switch :sw, criteria: :greater_than, threshold: 0.01
        scope :out
        connect :a, to: {:sw, 0}
        connect :ctrl, to: {:sw, 1}
        connect :b, to: {:sw, 2}
        connect :sw, to: :out, as: :y
      end

      y = model.run[:y]
      y.first.should eq(-10.0)
      y.last.should eq(10.0)
    end

    it "condition: (eeeval CondParser) gates by exact value" do
      model = CrySim.model "switch_condition" do
        duration 0.005
        dt 0.001
        constant :a, value: 1.0
        constant :b, value: 2.0
        step :mode, amplitude: 1.0, start_time: 0.002
        switch :sw, condition: "%.6f == 1.0"
        scope :out
        connect :a, to: {:sw, 0}
        connect :mode, to: {:sw, 1}
        connect :b, to: {:sw, 2}
        connect :sw, to: :out, as: :y
      end

      y = model.run[:y]
      y.first.should eq(2.0)
      y.last.should eq(1.0)
    end
  end

  describe "v0.3: discretize sugar" do
    it "matches manually calling sample() + ss(sys:) on the same continuous block" do
      tau = 0.5
      model = CrySim.model "discretize_test" do
        duration 1.0
        dt 0.01
        step :u, amplitude: 1.0
        tf :plant_c, num: [1.0], den: [tau, 1.0]
        discretize :plant_d, from: :plant_c, dt: 0.01, method: :zoh
        scope :out
        connect :u, to: :plant_c
        connect :u, to: :plant_d
        connect :plant_c, to: :out, as: :y_continuous
        connect :plant_d, to: :out, as: :y
      end
      result = model.run

      manual_sys = CrySpace::TransferFunction.new([1.0].to_tensor, [tau, 1.0].to_tensor)
        .to_statespace.sample(0.01, method: :zoh)
      manual_model = CrySim.model "manual" do
        duration 1.0
        dt 0.01
        step :u, amplitude: 1.0
        ss :plant, sys: manual_sys
        scope :out
        connect :u, to: :plant
        connect :plant, to: :out, as: :y
      end
      manual_result = manual_model.run

      result[:y].each_with_index { |v, i| manual_result[:y][i].should be_close(v, 1e-12) }
    end

    it "rejects discretizing an already-discrete or non-LTI block" do
      expect_raises(CrySim::ModelError, /already discrete/) do
        CrySim.model "already_discrete" do
          duration 0.1
          dt 0.01
          step :u
          dss :d1, a: [[0.9]], b: [[1.0]], c: [[1.0]], d: [[0.0]], dt: 0.01
          discretize :d2, from: :d1, dt: 0.01
        end
      end

      expect_raises(CrySim::ModelError, /no state-space representation/) do
        CrySim.model "no_state_space" do
          duration 0.1
          dt 0.01
          gain :g, k: 2.0
          discretize :gd, from: :g, dt: 0.01
        end
      end
    end
  end

  describe "v0.3: scoped LTI fast-path (run_fast / to_state_space)" do
    it "matches the general engine to floating-point precision on a 3-block series chain" do
      model = CrySim.model "fastpath_test" do
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

      slow = model.run
      fast = model.run_fast
      slow[:y].each_with_index { |v, i| fast[:y][i].should be_close(v, 1e-9) }
    end

    it "rejects a chain containing a non-LTI block" do
      model = CrySim.model "with_gain" do
        duration 1.0
        dt 0.01
        step :u
        gain :g, k: 2.0
        tf :plant, num: [1.0], den: [1.0, 1.0]
        scope :out
        connect :u, to: :g
        connect :g, to: :plant
        connect :plant, to: :out, as: :y
      end
      expect_raises(CrySim::ModelError, /is not LTI/) { model.to_state_space }
    end

    it "rejects a chain with feedback" do
      model = CrySim.model "with_feedback" do
        duration 1.0
        dt 0.01
        step :ref
        sum :err, signs: "+-"
        tf :plant, num: [1.0], den: [1.0, 1.0]
        scope :out
        connect :ref, to: {:err, 0}
        feedback from: :plant, to: {:err, 1}
        connect :err, to: :plant
        connect :plant, to: :out, as: :y
      end
      expect_raises(CrySim::ModelError, /feedback wires are not supported/) { model.to_state_space }
    end

    it "rejects fan-out in the middle of the chain" do
      model = CrySim.model "with_fanout" do
        duration 1.0
        dt 0.01
        step :u
        tf :stage1, num: [1.0], den: [1.0, 1.0]
        tf :stage2a, num: [1.0], den: [1.0, 1.0]
        tf :stage2b, num: [1.0], den: [1.0, 1.0]
        scope :out
        connect :u, to: :stage1
        connect :stage1, to: :stage2a
        connect :stage1, to: :stage2b
        connect :stage2a, to: :out, as: :a
        connect :stage2b, to: :out, as: :b
      end
      expect_raises(CrySim::ModelError, /exactly one outgoing wire/) { model.to_state_space }
    end
  end

  describe "NaN/Inf detection with context" do
    it "raises NonFiniteValueError naming the block, port and time that produced it" do
      model = CrySim.model "blowup" do
        duration 0.05
        dt 0.01
        ramp :u, slope: 1.0, start_time: 0.0
        fn :bad, expr: "1/(u-0.02)"
        scope :out
        connect :u, to: :bad
        connect :bad, to: :out, as: :y
      end

      expect_raises(CrySim::NonFiniteValueError, /block :bad produced/) do
        model.run
      end

      begin
        model.run
      rescue ex : CrySim::NonFiniteValueError
        ex.block_name.should eq("bad")
        ex.port.should eq(0)
        ex.time.should be_close(0.02, 1e-9)
      end
    end

    it "render_error highlights the offending block in the diagram" do
      model = CrySim.model "blowup2" do
        duration 0.05
        dt 0.01
        ramp :u, slope: 1.0
        fn :bad, expr: "1/(u-0.02)"
        scope :out
        connect :u, to: :bad
        connect :bad, to: :out, as: :y
      end

      begin
        model.run
        fail "expected NonFiniteValueError"
      rescue err : CrySim::NonFiniteValueError
        svg = CrySim::Diagram::SvgRenderer.new(model, highlight: err.block_name).to_svg
        svg.should contain("blk-error")
      end
      LayoutCheck.assert_no_overlaps(model)
    end
  end

  describe "equivalence against cryspace's own step_response/impulse_response" do
    it "matches cryspace's step_response on a first-order system" do
      tau = 2.0
      model = CrySim.model "equiv1" do
        duration 10.0
        dt 0.1
        step :u, amplitude: 1.0
        tf :plant, num: [1.0], den: [tau, 1.0]
        scope :out
        connect :u, to: :plant
        connect :plant, to: :out, as: :y
      end
      result = model.run
      _, _, y_arr = model.state_space_of(:plant).step_response(n_steps: 101)
      result[:y].each_with_index { |v, i| y_arr[i][0, 0].value.should be_close(v, 1e-6) }
    end

    it "matches cryspace's step_response on an underdamped second-order system" do
      model = CrySim.model "equiv2" do
        duration 10.0
        dt 0.1
        step :u, amplitude: 1.0
        tf :plant, num: [4.0], den: [1.0, 0.4, 4.0]
        scope :out
        connect :u, to: :plant
        connect :plant, to: :out, as: :y
      end
      result = model.run
      _, _, y_arr = model.state_space_of(:plant).step_response(n_steps: 101)
      result[:y].each_with_index { |v, i| y_arr[i][0, 0].value.should be_close(v, 1e-3) }
    end

    it "impulse response with method: :euler matches the analytic first-order response" do
      # h(t) = (1/tau) * exp(-t/tau)
      tau = 2.0
      model = CrySim.model "impulse_euler" do
        duration 5.0
        dt 0.001
        method :euler
        impulse :u, area: 1.0, time: 0.0
        tf :plant, num: [1.0], den: [tau, 1.0]
        scope :out
        connect :u, to: :plant
        connect :plant, to: :out, as: :y
      end
      result = model.run
      result.t.each_with_index do |t, i|
        next if t == 0.0 # y(0) is the pre-impulse value, not h(0+)
        analytic = (1.0 / tau) * Math.exp(-t / tau)
        result[:y][i].should be_close(analytic, 5e-3)
      end
    end

    # Documents a real, quantified characteristic found via this equivalence
    # check (see Blocks::Impulse's doc comment): RK4's k4 evaluation lands
    # exactly on the impulse's excluded right edge, so it always misses 1/6
    # of the weighted average — independent of dt. This is not "fixed" by a
    # boundary tweak (that just relocates the same bias into the next
    # step); the point of this test is to catch a *regression* in the
    # known, understood bias, not to bless it as correct.
    it "RK4's impulse response is biased to 5/6 of the analytic value, regardless of dt" do
      tau = 2.0
      [0.01, 0.001].each do |step_dt|
        model = CrySim.model "impulse_rk4_#{step_dt}" do
          duration 1.0
          dt step_dt
          impulse :u, area: 1.0, time: 0.0
          tf :plant, num: [1.0], den: [tau, 1.0]
          scope :out
          connect :u, to: :plant
          connect :plant, to: :out, as: :y
        end
        result = model.run
        analytic_at_dt = (1.0 / tau) * Math.exp(-step_dt / tau)
        result[:y][1].should be_close((5.0 / 6.0) * analytic_at_dt, 2e-3)
      end
    end
  end

  describe "lookup_table (1D)" do
    it "linearly interpolates between breakpoints" do
      model = CrySim.model "lut_test" do
        duration 0.04
        dt 0.01
        ramp :u, slope: 25.0
        lookup_table :lut, breakpoints: [0.0, 0.5, 1.0], values: [0.0, 100.0, 0.0]
        scope :out
        connect :u, to: :lut
        connect :lut, to: :out, as: :y
      end
      y = model.run[:y]
      y.map(&.round(3)).should eq([0.0, 50.0, 100.0, 50.0, 0.0])
    end

    it "extrapolates past the edges when extrapolate: true" do
      model = CrySim.model "lut_extrap" do
        duration 0.03
        dt 0.01
        ramp :u, slope: 100.0
        lookup_table :lut, breakpoints: [0.0, 1.0], values: [0.0, 10.0], extrapolate: true
        scope :out
        connect :u, to: :lut
        connect :lut, to: :out, as: :y
      end
      y = model.run[:y]
      y.map(&.round(3)).should eq([0.0, 10.0, 20.0, 30.0])
    end

    it "clamps to the edge value when extrapolate: false (default)" do
      model = CrySim.model "lut_clamped" do
        duration 0.03
        dt 0.01
        ramp :u, slope: 100.0
        lookup_table :lut, breakpoints: [0.0, 1.0], values: [0.0, 10.0]
        scope :out
        connect :u, to: :lut
        connect :lut, to: :out, as: :y
      end
      y = model.run[:y]
      y.map(&.round(3)).should eq([0.0, 10.0, 10.0, 10.0])
    end

    it "rejects a non-increasing breakpoints array" do
      expect_raises(CrySim::ModelError, /strettamente crescenti/) do
        CrySim.model "bad_lut" do
          duration 0.1
          dt 0.01
          constant :u, value: 0.5
          lookup_table :lut, breakpoints: [0.0, 1.0, 0.5], values: [0.0, 1.0, 2.0]
        end
      end
    end
  end

  describe "rate_limiter" do
    it "ramps at exactly rising_rate * dt per step" do
      model = CrySim.model "rl_test" do
        duration 1.0
        dt 0.1
        step :u, amplitude: 10.0
        rate_limiter :rl, rising_rate: 2.0
        scope :out
        connect :u, to: :rl
        connect :rl, to: :out, as: :y
      end
      y = model.run[:y]
      y.map(&.round(6)).should eq((0..10).map { |i| (i * 0.2).round(6) })
    end

    it "uses an asymmetric falling_rate when given, defaulting to -rising_rate otherwise" do
      model = CrySim.model "rl_asym" do
        duration 0.6
        dt 0.1
        step :u, amplitude: 1.0, start_time: 0.1
        step :u2, amplitude: -1.0, start_time: 0.3
        sum :combined, signs: "++"
        rate_limiter :rl, rising_rate: 100.0, falling_rate: -0.5
        scope :out
        connect :u, to: {:combined, 0}
        connect :u2, to: {:combined, 1}
        connect :combined, to: :rl
        connect :rl, to: :out, as: :y
      end
      y = model.run[:y]
      y.map(&.round(3)).should eq([0.0, 0.0, 1.0, 1.0, 0.95, 0.9, 0.85])
    end
  end

  describe "method: :midpoint" do
    it "converges to the analytic impulse response as dt -> 0 (unlike :rk4's fixed 5/6 bias)" do
      tau = 2.0
      [0.01, 0.001].each do |step_dt|
        model = CrySim.model "midpoint_impulse_#{step_dt}" do
          duration 0.5
          dt step_dt
          method :midpoint
          impulse :u, area: 1.0, time: 0.0
          tf :plant, num: [1.0], den: [tau, 1.0]
          scope :out
          connect :u, to: :plant
          connect :plant, to: :out, as: :y
        end
        result = model.run
        analytic = (1.0 / tau) * Math.exp(-step_dt / tau)
        # error should shrink roughly linearly with dt, not sit at a fixed
        # fraction like RK4's 5/6 -- a loose bound confirms convergence
        # without pinning down the exact constant.
        result[:y][1].should be_close(analytic, 2.0 * step_dt)
      end
    end

    it "matches the analytic step response closely (2nd-order accurate)" do
      tau = 2.0
      model = CrySim.model "midpoint_step" do
        duration 10.0
        dt 0.01
        method :midpoint
        step :u, amplitude: 1.0
        tf :plant, num: [1.0], den: [tau, 1.0]
        scope :out
        connect :u, to: :plant
        connect :plant, to: :out, as: :y
      end
      result = model.run
      result.t.each_with_index do |t, i|
        result[:y][i].should be_close(1.0 - Math.exp(-t / tau), 1e-5)
      end
    end

    it "rejects an unknown solver method" do
      expect_raises(CrySim::ModelError, /unknown solver method/) do
        CrySim.model "bad_method" do
          duration 0.1
          dt 0.01
          method :foo
        end
      end
    end
  end
end
