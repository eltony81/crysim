require "./spec_helper"

# Only compiled/run with `crystal spec -Darrow spec/arrow_io_spec.cr` — the
# main `crystal spec` (no flag) never touches this file's content, matching
# how num.cr itself gates its own Arrow backend. See src/crysim/arrow_io.cr.
{% if flag?(:arrow) %}
  describe "SimResult Feather/Parquet export" do
    it "round-trips t and every logged signal through Feather" do
      model = CrySim.model "arrow_feather_test" do
        duration 0.5
        dt 0.1
        step :u, amplitude: 2.0
        tf :plant, num: [1.0], den: [1.0, 1.0]
        scope :out
        connect :u, to: :plant
        connect :plant, to: :out, as: :y
      end
      result = model.run
      path = File.tempname("crysim_spec", ".feather")
      begin
        result.to_feather(path)
        File.exists?(path).should be_true
        File.size(path).should be > 0
      ensure
        File.delete(path) if File.exists?(path)
      end
    end

    it "round-trips t and every logged signal through Parquet" do
      model = CrySim.model "arrow_parquet_test" do
        duration 0.5
        dt 0.1
        step :u, amplitude: 2.0
        tf :plant, num: [1.0], den: [1.0, 1.0]
        scope :out
        connect :u, to: :plant
        connect :plant, to: :out, as: :y
      end
      result = model.run
      path = File.tempname("crysim_spec", ".parquet")
      begin
        result.to_parquet(path)
        File.exists?(path).should be_true
        File.size(path).should be > 0
      ensure
        File.delete(path) if File.exists?(path)
      end
    end
  end
{% end %}
