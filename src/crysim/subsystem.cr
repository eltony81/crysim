module CrySim
  # A reusable, parametric block template (v0.2). Defined once with
  # `CrySim.subsystem`, instantiated any number of times with `use` inside
  # a model, each instance getting its own prefixed set of blocks.
  #
  # Crystal blocks captured as a Proc can't retroactively rebind their
  # implicit receiver the way Ruby's `instance_eval` does (the `with X
  # yield` sugar only works for a block literal passed directly to the
  # method that receives it, evaluated immediately — not for one stored
  # and invoked later against a different, not-yet-existing builder). The
  # template body therefore takes the builder explicitly as `sub`:
  #
  #   motor_stage = CrySim.subsystem("dc_motor") do |sub, params|
  #     sub.tf :dynamics, num: [params[:k]], den: [params[:tau], 1.0, 0.0]
  #     sub.inport  :v_in,  to: :dynamics
  #     sub.outport :theta, from: :dynamics
  #   end
  #
  #   CrySim.model "two_motors" do
  #     use motor_stage, as: :m1, k: 2.0, tau: 0.5
  #     use motor_stage, as: :m2, k: 1.5, tau: 0.3
  #     # :m1 / :m2 are now usable as a single in/out port each:
  #     connect :ref, to: :m1
  #     connect :m1,  to: :out, as: :theta1
  #   end
  class Subsystem
    getter name : String

    def initialize(@name : String, &@block : ModelBuilder, Hash(Symbol, Float64) -> Nil)
    end

    # Inlines the template into `builder` under `prefix`, with `params`
    # bound for this instance. See ModelBuilder#use/#begin_subsystem.
    def instantiate(builder : ModelBuilder, prefix : Symbol, params : Hash(Symbol, Float64))
      builder.begin_subsystem(prefix)
      @block.call(builder, params)
      builder.end_subsystem(prefix)
    end
  end

  def self.subsystem(name : String, &block : ModelBuilder, Hash(Symbol, Float64) -> Nil) : Subsystem
    Subsystem.new(name, &block)
  end
end
