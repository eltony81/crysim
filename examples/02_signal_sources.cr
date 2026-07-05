require "../src/crysim"

# Tour of the signal sources: classic waveforms, an eeeval expression
# source, and a custom Fn block filtering through a first-order plant.

model = CrySim.model "signal_sources" do
  duration 4.0
  dt 0.001

  sine :sin_w, amplitude: 1.0, freq: 1.0
  pulse :sq_w, amplitude: 1.0, period: 1.0, duty: 0.3
  triangle :tri_w, amplitude: 1.0, period: 1.0
  chirp :sweep, amplitude: 1.0, f0: 0.5, f1: 4.0, t1: 4.0
  noise :dist, sigma: 0.05, seed: 42

  # eeeval expression source: saturated ramp + tone
  signal :ref, expr: "0.5*t + 0.05*sin(2*pi*10*t)"

  # noisy sine through a low-pass plant, squared by a custom expression
  sum :mix, signs: "++"
  tf :lp, num: [1.0], den: [0.05, 1.0]
  fn :sq, expr: "u^2"
  scope :waves, title: "Generatori"
  scope :chain, title: "Catena rumore -> filtro -> quadrato"

  connect :sin_w, to: {:mix, 0}
  connect :dist, to: {:mix, 1}
  connect :mix, to: :lp
  connect :lp, to: :sq

  connect :sq_w, to: :waves
  connect :tri_w, to: :waves
  connect :sweep, to: :waves
  connect :ref, to: :waves

  connect :mix, to: :chain, as: :noisy
  connect :lp, to: :chain, as: :filtered
  connect :sq, to: :chain, as: :squared
end

result = model.run
puts "logged: #{result.keys.join(", ")}"
result.plot("signal_sources.html")
model.render("signal_sources_diagram.html")
puts "written: signal_sources.html, signal_sources_diagram.html"
