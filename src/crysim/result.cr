require "json"

module CrySim
  # A sink's view over the logged signals (used to group plot panels).
  record ScopeView, title : String, keys : Array(String)

  # Display metadata for a logged signal: its role (:input, :output,
  # :monitor, ...) and an optional human-readable caption.
  record SignalMeta, role : Symbol, display : String?

  # Output of Model#run: the time vector plus every logged signal
  # (labeled wires, probes, scope inputs). Signals are keyed by String
  # internally (a subsystem-internal signal's name may be a prefixed
  # composite like "m1.dynamics" — see Wire's doc comment for why that
  # can't be a Symbol) — but every accessor also takes a plain Symbol for
  # the common case, so `result[:position]` keeps working unchanged.
  class SimResult
    getter model_name : String
    getter t : Array(Float64)
    getter signals : Hash(String, Array(Float64))
    getter scopes : Array(ScopeView)
    getter meta : Hash(String, SignalMeta)

    def initialize(@model_name : String, @t : Array(Float64), @signals : Hash(String, Array(Float64)),
                   @scopes : Array(ScopeView), @meta : Hash(String, SignalMeta) = {} of String => SignalMeta)
    end

    # Human-readable name for a logged signal: its display caption (or raw
    # name) followed by its role, e.g. "Posizione motore (output)".
    def display_name(key : String) : String
      m = @meta[key]?
      base = m.try(&.display) || key
      "#{base} (#{(m.try(&.role) || :signal)})"
    end

    def display_name(key : Symbol) : String
      display_name(key.to_s)
    end

    def [](key : String) : Array(Float64)
      @signals[key]? || raise KeyError.new("no logged signal :#{key} (available: #{@signals.keys.join(", ")})")
    end

    def [](key : Symbol) : Array(Float64)
      self[key.to_s]
    end

    def keys : Array(String)
      @signals.keys
    end

    # Bridge to num.cr for further analysis.
    def tensor(key : String) : Float64Tensor
      self[key].to_tensor
    end

    def tensor(key : Symbol) : Float64Tensor
      tensor(key.to_s)
    end

    def time_tensor : Float64Tensor
      @t.to_tensor
    end

    def to_csv(path : String)
      ks = @signals.keys
      File.open(path, "w") do |f|
        f << "t"
        ks.each { |k| f << "," << k }
        f << "\n"
        @t.each_with_index do |ti, i|
          f << ti
          ks.each { |k| f << "," << @signals[k][i] }
          f << "\n"
        end
      end
    end

    # Interactive HTML plot, one panel per scope (plus one panel with the
    # remaining logged signals). Same Chart.js template used by cryspace's
    # step_plot/bode_plot for a consistent look across the ecosystem.
    def plot(path : String)
      File.write(path, wrap_html(panels_html, max_width: 900))
    end

    # Combined report: the model's SVG block diagram (with a small
    # sparkline of each logged signal drawn next to its wire label) above
    # the same plot panels as `#plot`. One file, diagram and data together.
    def report(model : Model, path : String)
      diagram = Diagram::SvgRenderer.new(model, self).to_svg
      body = <<-BODY
        <div class="panel diagram-panel">
          <h2>Block diagram</h2>
          #{diagram}
        </div>
        #{panels_html}
      BODY
      File.write(path, wrap_html(body, max_width: 1100))
    end

    private def panels_html : String
      palette = ["#38bdf8", "#f472b6", "#4ade80", "#facc15", "#c084fc", "#fb923c"]
      shown = Set(String).new
      panels = [] of Tuple(String, Array(String))
      @scopes.each do |s|
        panels << {s.title, s.keys}
        s.keys.each { |k| shown << k }
      end
      rest = @signals.keys.reject { |k| shown.includes?(k) }
      panels << {"Signals", rest} unless rest.empty?

      times_json = @t.map(&.round(6)).to_json
      String.build do |html|
        panels.each_with_index do |(title, keys), pi|
          datasets = keys.map_with_index do |k, i|
            {
              "label"           => JSON::Any.new(display_name(k)),
              "data"            => JSON::Any.new(self[k].map { |v| JSON::Any.new(v) }),
              "borderColor"     => JSON::Any.new(palette[i % palette.size]),
              "backgroundColor" => JSON::Any.new("transparent"),
              "borderWidth"     => JSON::Any.new(2.0),
              "pointRadius"     => JSON::Any.new(0.0),
              "tension"         => JSON::Any.new(0.1),
            }
          end
          html << <<-PANEL
            <div class="panel">
              <h2>#{title}</h2>
              <canvas id="chart#{pi}"></canvas>
            </div>
            <script>
              new Chart(document.getElementById('chart#{pi}').getContext('2d'), {
                type: 'line',
                data: { labels: #{times_json}, datasets: #{datasets.to_json} },
                options: {
                  responsive: true,
                  animation: false,
                  scales: {
                    x: { title: { display: true, text: 'Time (s)', color: '#94a3b8' },
                         grid: { color: '#334155' }, ticks: { color: '#94a3b8', maxTicksLimit: 15 } },
                    y: { grid: { color: '#334155' }, ticks: { color: '#94a3b8' } }
                  },
                  plugins: { legend: { labels: { color: '#f8fafc' } } }
                }
              });
            </script>
          PANEL
        end
      end
    end

    private def wrap_html(body : String, max_width : Int32) : String
      <<-HTML
      <!DOCTYPE html>
      <html>
      <head>
        <title>CrySim - #{@model_name}</title>
        <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
        <style>
          body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 40px; background: #0f172a; color: #f8fafc; }
          .container { max-width: #{max_width}px; margin: 0 auto; overflow-x: auto; }
          .panel { background: #1e293b; padding: 30px; border-radius: 12px; box-shadow: 0 10px 25px rgba(0,0,0,0.5); margin-bottom: 30px; }
          .diagram-panel { overflow-x: auto; }
          h1 { color: #38bdf8; font-weight: 400; text-align: center; }
          h2 { color: #94a3b8; font-weight: 400; font-size: 18px; margin-top: 0; }
          .footer { font-size: 12px; color: #64748b; text-align: center; margin-top: 20px; }
        </style>
      </head>
      <body>
        <div class="container">
          <h1>#{@model_name}</h1>
          #{body}
          <div class="footer">Generated by CrySim v#{VERSION}</div>
        </div>
      </body>
      </html>
      HTML
    end
  end
end
