# CrySim — Analisi di fattibilità e progettazione

**Una libreria di simulazione a blocchi ispirata a Simulink, in Crystal, costruita sopra `eltony81/cryspace`, `eltony81/num.cr` e `eltony81/easy_expression_eval`**

Data analisi: 2026-07-04
Versioni di riferimento: cryspace **1.28.0**, num.cr **1.31.1**, eeeval **1.1.0**

> **Stato**: la v0.1 descritta qui è implementata in questo repository (2026-07-04):
> DSL `connect`, motore Euler/RK4, ~25 blocchi, espressioni eeeval, probe inline,
> renderer SVG, SimResult con plot Chart.js e CSV. Vedi `README.md`, `examples/`, `spec/`.
> Aggiunta successiva: ogni segnale loggato porta un **ruolo** (`input`/`output`/`monitor`,
> inferito dalla categoria del blocco sorgente, sovrascrivibile con `role:`) e una
> **display label** opzionale (`display:`) come metadato del wire — entrambi mostrati
> nell'etichetta colorata del filo nell'SVG e come nome nella legenda del grafico.
>
> **v0.2 implementata**: sugar `>>` e `feedback from:/to:`, sottosistemi parametrici
> (`CrySim.subsystem`/`use`, SISO), report HTML unificato con sparkline sui fili. Rimandati
> a v0.3 con motivazione esplicita: fast-path di appiattimento LTI e segnali vettoriali
> (Mux/Demux) — dettagli in `PIANO_FEATURE.md`. Durante l'implementazione dei sottosistemi
> è emerso un vincolo del linguaggio non ovvio: **Crystal non permette di creare Symbol a
> runtime** (nessun `String#to_sym`), quindi l'identità interna di blocchi/segnali è
> passata da `Symbol` a `String`, con overload che accettano `Symbol` in ogni punto
> d'accesso pubblico (`result[:position]` continua a funzionare). Vedi `PIANO_FEATURE.md`
> §A per i dettagli, incluso un bug preesistente in v0.1 scoperto e corretto nello stesso
> passaggio (auto-naming di porte MIMO senza `as:` esplicito).
>
> **v0.3 implementata**: multi-rate semplice (`dss` rispetta il proprio `dt`, `pid rate:`),
> con un bug latente corretto nello stesso passaggio (il motore aggiornava prima ogni
> blocco campionato ad ogni passo base, ignorando il suo rate dichiarato); `unit_delay`;
> `discretize` (sugar continuo→discreto via `sample` di cryspace); `switch`
> (`criteria:`/`threshold:` in Crystal puro, oppure `condition:` via `EEEval::CondParser`
> per selezione a valore esatto — il tokenizer di CondParser non riconosce affatto `>`/`<`,
> verificato empiricamente); fast-path LTI **scoped** (`model.to_state_space`/`run_fast`)
> per il caso sorgente→catena LTI→sink senza diramazioni/feedback, verificato a precisione
> di macchina (~1e-15) contro il motore generale. Rimandati a v0.4, invariati rispetto alla
> valutazione v0.2: segnali vettoriali (Mux/Demux), sottosistemi multi-porta, e
> l'appiattimento LTI generale (diramazioni/feedback) — dettagli in `PIANO_FEATURE.md`.

---

## 1. Obiettivo e visione

CrySim vuole offrire quello che Simulink offre rispetto a MATLAB: mentre cryspace fornisce gli *strumenti di analisi* (state-space, transfer function, LQR, Kalman, solver ODE...), CrySim fornisce la *composizione visuale-testuale della simulazione*: un modello descritto come **diagramma a blocchi interconnessi**, dove l'utente dichiara blocchi, segnali e connessioni tramite un **DSL** e il motore si occupa di ordinamento, integrazione numerica e raccolta dei risultati.

Il principio guida della prima versione è: **nessuna nuova matematica**. Ogni blocco del set minimo deve essere implementabile con funzionalità già presenti in num.cr e cryspace. CrySim v0.1 è quindi essenzialmente un *orchestratore* + un *linguaggio*.

```
┌─────────────────────────────────────────────┐
│  CrySim DSL        (composizione modello)   │   ← il cuore del progetto
├─────────────────────────────────────────────┤
│  CrySim Engine     (grafo, sort, stepping)  │
├─────────────────────────────────────────────┤
│  cryspace          (StateSpace, TF, Solver, │
│                     PID, feedback, plot)    │
├─────────────────────────────────────────────┤
│  num.cr            (Tensor, BLAS/LAPACK,    │
│                     Arrow SIMD, OpenCL)     │
└─────────────────────────────────────────────┘
```

---

## 2. Inventario delle fondamenta disponibili

### 2.1 Cosa offre num.cr (v1.31.1) di utile a CrySim

| Funzionalità | Uso in CrySim |
|---|---|
| `Tensor(Float64, CPU)` n-dimensionale | Tipo universale dei segnali e dei log di simulazione |
| BLAS/LAPACK (`matmul`, `solve`, `eig_c`, ...) | Già usati indirettamente via cryspace |
| Backend **ARROW** con SIMD (`-Darrow`) | Fast-path per simulazioni lunghe di modelli LTI appiattiti |
| `Arrow::FeatherWriter` / `ParquetWriter` | Blocco sink "ToFile": logging colonnare ultra-veloce dei run |
| Dispatch dinamico CPU/Arrow/OpenCL per soglia | Gratis, nessun lavoro lato CrySim |

### 2.2 Cosa offre cryspace (v1.28.0) di utile a CrySim

| API cryspace | Blocco / funzione CrySim che ne deriva |
|---|---|
| `StateSpace.new(a, b, c, d, dt)` | Blocco **StateSpace** (continuo e discreto) |
| `TransferFunction.new(num, den, dt)` + conversione a SS | Blocco **TransferFcn** |
| `simulate(t, x0, u, method: :rk4)` / `lsim` | Esecuzione fast-path di sottosistemi LTI |
| Connessioni `*` (serie), `+` (parallelo), `feedback` | **Appiattimento** di sotto-diagrammi LTI in un unico StateSpace |
| `Solver.euler / rk4 / rk45` (Proc-based) | Motore di integrazione a passo fisso del diagramma generale |
| `PIDController` (anti-windup, saturazione attuatore) | Blocco **PID** |
| `Nonlinear` (saturation, deadzone) | Blocchi **Saturation**, **DeadZone** |
| `step_plot`, `bode_plot` (output HTML) | Blocco sink **Scope** (riuso del renderer HTML) |
| `sample(dt)` ZOH/Tustin, `to_continuous` | Coesistenza blocchi continui/discreti (fase 2) |
| Kalman / Luenberger / UKF / EKF | Blocchi **Observer** (fase 2/3) |

### 2.3 Cosa offre easy_expression_eval (eeeval) di utile a CrySim

Terza libreria della famiglia (`eltony81/easy_expression_eval`), piccola ma strategica:

| Funzionalità eeeval | Uso in CrySim |
|---|---|
| `CalcFuncParser.compile(expr)` → AST riusabile | Sorgenti e blocchi Fn definiti come **stringhe** compilate una volta al `build`, valutate a ogni passo senza re-parsing |
| Variabili native (`{"t" => t, "u" => u}`) senza string-replace | Binding efficiente di tempo e segnali nelle espressioni |
| Funzioni matematiche (`sin cos tan exp log sqrt abs floor ...`) e costanti (`pi`, `e`, `tau`) | Copertura completa dei segnali di ingresso analitici |
| `CondParser` (booleani, `&&`/`\|\|`, confronti) | Blocchi condizionali **Switch/If** (fase 2) |

Il valore architetturale va oltre la comodità: una `Proc` Crystal **non è serializzabile**, una stringa sì. Le espressioni eeeval sono ciò che rende possibile un modello **completamente dichiarativo e caricabile a runtime** (il loader YAML della roadmap) senza ricompilare nulla — un file `.yaml` può descrivere anche i segnali custom.

**Conclusione dell'inventario**: tutto il necessario per un set minimo di blocchi esiste già. Il lavoro nuovo è ~90% design del DSL e del grafo, ~10% colla numerica.

---

## 3. Concetti architetturali fondamentali

### 3.1 Modello a oggetti del core

```
Model
 ├── blocks : Array(Block)          # nodi del grafo
 ├── connections : Array(Wire)      # archi orientati (out_port → in_port)
 ├── solver_config : SolverConfig   # metodo (:euler/:rk4), dt, t_span
 └── run() : SimResult

Block (abstract)
 ├── name : Symbol
 ├── n_inputs / n_outputs
 ├── direct_feedthrough? : Bool     # l'uscita dipende istantaneamente dall'ingresso?
 ├── state : Float64Tensor?         # nil per blocchi senza stato (Gain, Sum...)
 ├── output(t, x, u) : Float64Tensor       # equazione di uscita
 └── derivative(t, x, u) : Float64Tensor?  # equazione di stato (blocchi continui)

SimResult
 ├── t : Float64Tensor
 ├── signals : Hash(Symbol, Float64Tensor)   # ogni segnale nominato loggato
 └── to_feather(path) / plot(path)
```

La separazione `output` / `derivative` è la stessa astrazione dei *S-function* di Simulink e si sposa perfettamente con i solver Proc-based di `CrySpace::Solver`: il motore costruisce un unico `Proc(Float64Tensor, Float64, Float64Tensor)` che concatena gli stati di tutti i blocchi e lo passa a `rk4`.

### 3.2 Il motore di esecuzione: due strategie complementari

**Strategia A — Co-simulazione generale (v0.1, obbligatoria).**
Ad ogni passo di integrazione:

1. Valutazione delle uscite dei blocchi in **ordine topologico** calcolato sul sotto-grafo dei blocchi con *direct feedthrough* (i blocchi con stato — Integrator, StateSpace — rompono i cicli, esattamente come in Simulink).
2. Calcolo delle derivate di tutti i blocchi con stato.
3. Avanzamento con Euler/RK4 riusando la struttura di `CrySpace::Solver` (stato globale = concatenazione degli stati dei blocchi).

Un ciclo interamente composto da blocchi feedthrough è un **loop algebrico**: in v0.1 viene rilevato al `build` e rifiutato con errore esplicativo (Simulink lo risolve iterativamente; è fuori scope per il set minimo).

**Strategia B — Appiattimento LTI (fast-path, v0.2).**
Se il diagramma (o un sotto-diagramma) contiene solo blocchi LTI (Gain, Sum, TransferFcn, StateSpace, Integrator), CrySim lo riduce a un singolo `CrySpace::StateSpace` usando gli operatori `*`, `+`, `feedback` già esistenti, e simula con `simulate` — che con tensori Arrow è già vettorizzato SIMD. Questo dà a CrySim prestazioni da libreria di controllo pura senza costo di dispatch per-blocco.

### 3.3 Semantica dei segnali (set minimo)

- Segnali **scalari** `Float64` per v0.1 (un filo = un valore per passo). I blocchi `StateSpace` MIMO espongono più porte.
- Rate unico: **un solo `dt` globale** a passo fisso. Multi-rate e RK45 adattivo rimandati (il solver `rk45` esiste già in cryspace, quindi la porta è aperta).
- Ogni filo può avere un nome: i fili nominati vengono loggati automaticamente in `SimResult`.

---

## 4. Il DSL — la parte più importante

### 4.1 Perché un DSL interno (embedded) e non un parser

Crystal offre tre meccanismi che rendono un DSL interno molto più conveniente di un file-format custom:

1. **Blocchi con `with ... yield`** — permettono `model do ... end` dove dentro il blocco sono visibili i metodi del builder senza receiver esplicito. È il meccanismo che rende naturale la sintassi dichiarativa.
2. **Macro** — validazioni a *compile time* (es. arità delle connessioni per blocchi builtin) e generazione di metodi-blocco senza boilerplate.
3. **Tipi e overload** — errori di composizione intercettati dal compilatore invece che a runtime.

Un DSL interno inoltre non preclude un formato file: una v futura può aggiungere un loader YAML/JSON che costruisce lo stesso grafo (utile per tool grafici).

### 4.2 Sintassi proposta

Stile primario — **dichiarativo a blocchi e connessioni**, il più vicino al mentale Simulink:

```crystal
require "crysim"

model = CrySim.model "dc_motor_position" do
  # ---- configurazione solver ----
  duration 5.0
  dt 0.001
  method :rk4

  # ---- blocchi ----
  step      :ref,   amplitude: 1.0, start_time: 0.1
  sum       :err,   signs: "+-"
  pid       :ctrl,  kp: 12.0, ki: 3.0, kd: 0.8, u_min: -24.0, u_max: 24.0
  tf        :motor, num: [2.0], den: [0.5, 1.0, 0.0]     # 2/(0.5s² + s)
  gain      :sensor, k: 1.0
  scope     :out,   title: "Risposta posizione"

  # ---- connessioni ----
  connect :ref,    to: {:err, 0}       # ingresso '+' del sommatore
  connect :sensor, to: {:err, 1}       # ingresso '-' (retroazione)
  connect :err,    to: :ctrl
  connect :ctrl,   to: :motor
  connect :motor,  to: :sensor
  connect :motor,  to: :out, as: :position   # filo nominato → loggato
  connect :ref,    to: :out
end

result = model.run
result.plot("position_response.html")       # riusa il renderer HTML di cryspace
result.to_feather("run_001.feather")        # Arrow FeatherWriter
puts result[:position].max
```

Zucchero sintattico complementare — **catene con `>>`** per i percorsi seriali, che riducono drasticamente i `connect` nel caso comune:

```crystal
CrySim.model "chain_style" do
  duration 5.0
  dt 0.001

  chain step(:ref, amplitude: 1.0) >> sum(:err, signs: "+-") >>
        pid(:ctrl, kp: 12.0, ki: 3.0) >> tf(:motor, num: [2.0], den: [0.5, 1.0, 0.0]) >>
        scope(:out)

  feedback from: :motor, to: {:err, 1}    # chiusura d'anello esplicita
end
```

I due stili coesistono: `>>` è solo sugar che genera le stesse `Wire` del `connect` esplicito. La retroazione resta sempre esplicita — è la scelta di design che tiene il DSL leggibile (in Simulink il filo di feedback è comunque il "filo speciale" del diagramma).

### 4.3 Sottosistemi e componibilità

La caratteristica che dà valore a lungo termine — i sottosistemi riusabili come in Simulink le subsystem/library:

```crystal
# definizione riusabile e parametrica
motor_stage = CrySim.subsystem "dc_motor" do |params|
  tf :dynamics, num: [params[:k]], den: [params[:tau], 1.0, 0.0]
  inport  :v_in,  to: :dynamics
  outport :theta, from: :dynamics
end

CrySim.model "two_motors" do
  duration 2.0
  dt 0.001
  use motor_stage, as: :m1, k: 2.0, tau: 0.5
  use motor_stage, as: :m2, k: 1.5, tau: 0.3
  # ... i sottosistemi appaiono come blocchi con porte :v_in / :theta
end
```

In v0.1 basta l'inlining: `use` espande i blocchi del sottosistema nel grafo del padre con nomi prefissati (`:"m1.dynamics"`). Nessun motore gerarchico necessario.

### 4.4 Implementazione del DSL: schizzo tecnico

```crystal
module CrySim
  def self.model(name : String, &) : Model
    builder = ModelBuilder.new(name)
    with builder yield          # ← i metodi-blocco diventano "keyword" del DSL
    builder.build               # validazione: porte, loop algebrici, dt
  end

  class ModelBuilder
    # macro che genera un metodo-DSL per ogni blocco builtin,
    # eliminando il boilerplate e centralizzando la registrazione
    macro register_block(name, klass)
      def {{name.id}}(id : Symbol, **params)
        add_block({{klass}}.new(id, **params))
      end
    end

    register_block step,   Blocks::Step
    register_block gain,   Blocks::Gain
    register_block sum,    Blocks::Sum
    register_block tf,     Blocks::TransferFcn
    register_block ss,     Blocks::StateSpaceBlock
    register_block pid,    Blocks::PID
    # ...
  end
end
```

`build` esegue le validazioni a runtime-di-costruzione (una porta di ingresso con 0 o >1 fili, blocchi orfani, loop algebrici via DFS sul sotto-grafo feedthrough) e restituisce un `Model` immutabile. Errori con messaggi *didattici* ("l'ingresso 1 di :err non è connesso") sono parte del prodotto quanto la sintassi.

### 4.5 Estensibilità utente

Un blocco custom è una classe che eredita `Block` — l'equivalente delle S-function:

```crystal
class MyFriction < CrySim::Block
  def initialize(name, @mu : Float64)
    super(name, n_inputs: 1, n_outputs: 1, direct_feedthrough: true)
  end

  def output(t, x, u)
    Tensor.from_array [@mu * Math.tanh(u[0] * 100.0)]
  end
end

# nel DSL:  block :fric, MyFriction.new(:fric, mu: 0.3)
```

---

## 5. Set minimo di blocchi (v0.1) — mappatura implementativa

Criterio di selezione: con questi ~25 blocchi si costruiscono tutti gli esempi classici (anello PID, massa-molla-smorzatore, RLC, inseguimento con saturazione attuatore, identificazione in frequenza) e **ognuno mappa direttamente su codice esistente**. Le sorgenti sono volutamente abbondanti: costano pochissimo (una formula in `output(t)`) e sono ciò che rende utile la libreria dal primo giorno.

### Sorgenti (senza ingressi, senza stato)

Tutte le sorgenti deterministiche sono pure funzioni del tempo `f(t)` — implementazione a costo quasi nullo.

| Blocco | DSL | Implementazione / note |
|---|---|---|
| Constant | `constant :c, value: 2.0` | banale |
| Step | `step :s, amplitude:, start_time:` | banale |
| Ramp | `ramp :r, slope:, start_time:` | banale |
| Sine | `sine :w, amplitude:, freq:, phase:` | `Math.sin` |
| Cosine | `cosine :w, amplitude:, freq:, phase:` | `Math.cos` (equivale a sine con fase +π/2, ma esplicito è più leggibile) |
| Impulse (Dirac) | `impulse :d, area: 1.0, time: 0.0` | la delta ideale non esiste numericamente: impulso rettangolare di **un solo passo** `dt` con ampiezza `area/dt` — stessa convenzione di `impulse_response` in cryspace |
| Pulse / Square | `pulse :p, amplitude:, period:, duty: 0.5` | onda quadra con duty cycle |
| Sawtooth | `sawtooth :sw, amplitude:, period:` | dente di sega |
| Triangle | `triangle :tr, amplitude:, period:` | onda triangolare |
| Chirp | `chirp :ch, f0:, f1:, t1:` | sweep lineare in frequenza — l'ingresso naturale per il modulo `ident` di cryspace (identificazione in frequenza) |
| Noise | `noise :n, sigma:, seed: nil` | rumore bianco gaussiano via **alea** (già dipendenza di num.cr); `seed` per run riproducibili |
| **Signal (custom)** | `signal :src, ->(t : Float64) { ... }` | sorgente arbitraria da `Proc(Float64, Float64)` — il jolly che copre tutto il resto |

Esempio di sorgente custom nel DSL:

```crystal
# profilo di riferimento arbitrario: rampa saturata con disturbo sinusoidale
signal :ref, ->(t : Float64) do
  base = Math.min(t * 0.5, 2.0)
  base + 0.05 * Math.sin(2.0 * Math::PI * 10.0 * t)
end
```

La stessa sorgente può essere definita come **espressione stringa** via eeeval (§2.3) — compilata in AST una sola volta al `build`, valutata a ogni passo con `{"t" => t}`:

```crystal
signal :ref, expr: "0.5*t + 0.05*sin(2*pi*10*t)"
```

Le due forme convivono con trade-off chiari: la `Proc` è type-checked e più veloce (codice nativo), l'espressione è **serializzabile** (indispensabile per il loader YAML), modificabile senza ricompilare, e sufficiente per la stragrande maggioranza dei segnali analitici. Lo stesso doppio binario vale per il blocco `Fn`: `fn :sq, expr: "u^2"` con binding `{"u" => u, "t" => t}`.

**Nota su Noise e RK4**: il rumore va campionato **una volta per passo** e tenuto costante durante i sotto-passi di RK4 (ZOH), altrimenti l'integratore vede un segnale non riproducibile tra le valutazioni interne. È la scelta pragmatica standard (non un vero solver SDE, che resta fuori scope); va documentata.

### Matematica (feedthrough, senza stato)
| Blocco | DSL | Implementazione |
|---|---|---|
| Gain | `gain :k, k: 5.0` | moltiplicazione |
| Sum | `sum :e, signs: "+-"` | somma con segni, N ingressi |
| Product | `product :p, n_inputs: 2` | prodotto elemento per elemento |
| Saturation | `saturation :sat, min:, max:` | `clamp` (coerente con `Nonlinear`) |
| DeadZone | `deadzone :dz, threshold:` | logica di `Nonlinear.describing_function_deadzone` in versione elemento-per-elemento |
| **Fn (custom)** | `fn :sq, ->(u : Float64, t : Float64) { u * u }` | trasformazione arbitraria del segnale — la versione lambda del blocco custom di §4.5, per quando una classe è troppa cerimonia |

### Dinamica continua (con stato)
| Blocco | DSL | Implementazione |
|---|---|---|
| Integrator | `integrator :i, x0: 0.0` | caso speciale SS: A=0, B=1, C=1, D=0 |
| TransferFcn | `tf :g, num:, den:` | `CrySpace::TransferFunction` → conversione a SS |
| StateSpace | `ss :p, a:, b:, c:, d:` | `CrySpace::StateSpace` diretto |
| PID | `pid :c, kp:, ki:, kd:, u_min:, u_max:` | `CrySpace::PIDController` (già con anti-windup) |

### Strumentazione (pass-through, 1 in / 1 out)
| Blocco | DSL | Implementazione |
|---|---|---|
| Probe / Monitor | `probe :u_mon` | identità `y = u` + log automatico del segnale |

Il **Probe** è l'equivalente del test point / Scope inline di Simulink: un monitor **inseribile in qualunque punto della catena** senza alterare la dinamica (feedthrough puro, zero stato, costo nullo). Serve a rendere osservabile un segnale *interno* — ad esempio il comando tra PID e impianto — senza dover tirare un filo fino a uno Scope terminale né rinominare connessioni. È il blocco che brilla nello stile a catena:

```crystal
chain step(:ref) >> sum(:err, signs: "+-") >> pid(:ctrl, kp: 12.0) >>
      probe(:u_mon) >>                       # ← monitor del comando, inserito e basta
      tf(:motor, num: [2.0], den: [0.5, 1.0, 0.0]) >> scope(:out)
```

Ogni probe registra il proprio segnale in `SimResult#signals` (chiave = nome del probe) e ottiene automaticamente: il suo grafico nel report HTML, la sparkline sul diagramma (§6.5) e la colonna nel file Feather. Aggiungerlo/toglierlo non cambia in alcun modo i risultati numerici della simulazione.

### Sink (senza uscite)
| Blocco | DSL | Implementazione |
|---|---|---|
| Scope | `scope :out, title:` | log + renderer HTML riusato da `plot.cr` |
| ToFile | `to_file :log, path: "run.feather"` | `Arrow::FeatherWriter` |

Ogni segnale in ingresso a uno Scope, che attraversa un Probe, o con `as: :nome` finisce in `SimResult#signals` — il logging è il vero prodotto del run, lo Scope è solo una vista.

---

## 6. Generazione grafica del diagramma (HTML/SVG)

Il modello CrySim è un grafo esplicito (blocchi + fili): renderizzarlo è quindi una pura funzione `Model → SVG`, senza alcuna dipendenza dal motore di simulazione. È anche uno strumento di **verifica visiva della correttezza**: vedere il diagramma che il DSL ha effettivamente costruito è il modo più rapido per accorgersi di una connessione sbagliata.

### 6.1 Formato scelto: HTML self-contained con SVG inline

- **SVG inline in un file HTML** generato come stringa da puro Crystal, zero dipendenze esterne — la stessa filosofia già usata da cryspace per `step_plot`/`bode_plot` (output HTML autonomo apribile nel browser).
- Vettoriale: zoom senza perdita, testo selezionabile, stampabile, embeddabile in documentazione.
- L'HTML che avvolge l'SVG abilita l'interattività (hover, highlight) con poche righe di JS inline; l'SVG puro resta esportabile da solo (`model.to_svg : String`).

### 6.2 API proposta

```crystal
model.to_svg                       # String — solo l'SVG
model.render("diagram.html")       # HTML self-contained interattivo
result.report("report.html")       # diagramma + plot degli scope in un unico report
```

### 6.3 Indipendenza dal dialetto DSL

Il renderer opera sul **grafo `Model` costruito**, mai sul sorgente DSL. Poiché `chain`/`>>` e `feedback from:` sono puro zucchero sintattico che genera gli stessi oggetti `Block` e `Wire` del `connect` esplicito (§4.2), qualunque stile di composizione — `connect`, catene, mix dei due, sottosistemi inlined con `use`, o un futuro loader YAML — produce lo stesso grafo e quindi lo **stesso identico diagramma**. Vale anche il viceversa: il diagramma è la prova visiva che i due stili hanno costruito davvero il modello che si intendeva.

Unica sfumatura: il filo di retroazione. Con `feedback from: :motor, to: {:err, 1}` l'arco arriva al renderer già **marcato semanticamente** come feedback; con il solo `connect` viene comunque **riconosciuto come back-edge** dalla DFS (la stessa usata dal motore per l'ordinamento topologico). In entrambi i casi il filo viene instradato sotto il diagramma e colorato da retroazione — cambia solo la fonte dell'informazione, non il risultato.

### 6.4 Algoritmo di layout (Sugiyama semplificato)

Il diagramma di controllo tipico (5–30 blocchi) non richiede un layout engine generale; basta un **layered layout** in 4 passi:

1. **Rimozione archi di feedback** — già identificati dal motore (back-edge nella DFS del grafo); il grafo residuo è un DAG.
2. **Assegnazione layer** — longest-path dalle sorgenti: sorgenti a sinistra (layer 0), sink a destra. È esattamente la convenzione di lettura Simulink.
3. **Ordinamento verticale nel layer** — euristica del baricentro (media delle posizioni dei predecessori) per minimizzare gli incroci.
4. **Routing dei fili** — ortogonale (segmenti orizzontali/verticali con angoli raccordati); i fili di **feedback corrono sotto il diagramma**, come nella prassi dei diagrammi di controllo.

Complessità implementativa contenuta: per la taglia di grafi in questione i passi 2–3 sono poche decine di righe.

### 6.5 Glifi dei blocchi (convenzioni Simulink)

Ogni categoria ha il suo glifo riconoscibile, così il diagramma si legge a colpo d'occhio:

| Blocco | Glifo |
|---|---|
| Gain | **triangolo** con il valore di k dentro |
| Sum | **cerchio** con i segni `+`/`−` accanto alle porte |
| TransferFcn | rettangolo con la **frazione** num/den (es. `2 / (0.5s²+s)`) |
| Integrator | rettangolo con `1/s` |
| StateSpace | rettangolo con `ẋ=Ax+Bu` |
| PID | rettangolo con `PID(kp,ki,kd)` |
| Step/Sine/Ramp | rettangolo con **mini-icona della forma d'onda** |
| Saturation/DeadZone | rettangolo con la caratteristica ingresso-uscita stilizzata |
| Scope | rettangolo con schermo stilizzato |

I fili nominati (`as: :position`) portano l'**etichetta del segnale** sul filo.

### 6.6 La grafica come strumento di diagnosi

È qui che il rendering paga di più — le validazioni del `build` diventano visive:

- **porte non connesse** → disegnate in rosso con marker;
- **loop algebrici** → il ciclo incriminato evidenziato in arancione (invece del solo messaggio d'errore testuale); `model.render` funziona anche su un modello che non passa la validazione, proprio per fare debugging;
- **hover su un blocco** → tooltip con i parametri effettivi (quello che il DSL ha davvero costruito, non quello che si crede di aver scritto);
- dopo `run`: **sparkline dei segnali loggati direttamente sui fili** nel report HTML (fase 2) — diagramma e risultati nello stesso colpo d'occhio;
- i blocchi **Probe** (§5) hanno un glifo dedicato (pillola con mini-forma d'onda) e sono il punto d'aggancio naturale delle sparkline: dove c'è un probe, il report mostra il segnale misurato esattamente in quel punto della catena.

### 6.7 Alternativa considerata e scartata: export Graphviz DOT

Un `model.to_dot` costa mezz'ora ed è un buon fallback di debug, ma richiede Graphviz installato, non controlla i glifi (niente triangolo del gain, niente frazioni), e il layout `dot` gestisce male i feedback sotto il diagramma. Può esistere come extra, ma **il renderer SVG nativo è la scelta primaria**: il valore sta proprio nel far somigliare l'output a un diagramma di controllo tradizionale.

---

## 7. Struttura proposta del progetto

```
crysim/
├── shard.yml                  # deps: cryspace (che porta num.cr)
├── src/
│   ├── crysim.cr              # entry point + CrySim.model
│   └── crysim/
│       ├── block.cr           # Block astratto, Port, Wire
│       ├── model.cr           # Model, validazioni, topological sort
│       ├── builder.cr         # ModelBuilder + macro DSL
│       ├── engine.cr          # loop di simulazione, ponte verso CrySpace::Solver
│       ├── result.cr          # SimResult, plot, feather, report HTML
│       ├── diagram/
│       │   ├── layout.cr      # layer assignment, barycenter, wire routing
│       │   ├── glyphs.cr      # glifi SVG per categoria di blocco
│       │   └── svg_renderer.cr# Model → SVG / HTML interattivo
│       └── blocks/
│           ├── sources.cr     # Constant, Step, Ramp, Sine
│           ├── math.cr        # Gain, Sum, Product, Saturation, DeadZone
│           ├── continuous.cr  # Integrator, TransferFcn, StateSpaceBlock, PID
│           └── sinks.cr       # Scope, ToFile
├── examples/
│   ├── 01_pid_loop.cr
│   ├── 02_mass_spring_damper.cr
│   └── 03_saturated_tracking.cr
└── spec/
```

`shard.yml`:

```yaml
dependencies:
  cryspace:
    github: eltony81/cryspace
  eeeval:
    github: eltony81/easy_expression_eval
```

---

## 8. Roadmap

> **Nota**: la tabella sotto è il piano *originale*, scritto prima di iniziare v0.2, e non
> viene più aggiornata (alcune voci sono ormai imprecise, es. Mux/Demux era previsto per
> v0.3 ma è stato rimandato a v0.4). Per lo stato attuale — cosa è stato consegnato in
> quale fase, cosa resta in backlog e perché — vedi **[PIANO_FEATURE.md](PIANO_FEATURE.md)**,
> l'unica fonte aggiornata.

| Fase | Contenuto | Dipende da |
|---|---|---|
| **v0.1 — MVP** | Core (Block/Wire/Model), DSL dichiarativo `connect`, motore co-simulazione Euler/RK4 a passo fisso, i ~25 blocchi del §5 (sorgenti complete, Fn custom, Probe inline, espressioni `expr:` via eeeval), SimResult + plot HTML + Feather, rilevamento loop algebrici, **renderer SVG base** (layout a layer, glifi standard, evidenziazione errori), 3 esempi | solo cryspace/num.cr esistenti |
| **v0.2 — Qualità e velocità** ✅ | Sugar `>>`/`feedback`, sottosistemi con `use`/inlining (SISO), **report HTML unificato** (diagramma + sparkline dei segnali sui fili + pannelli Chart.js). Rimandati a v0.3 (dettagli sotto): fast-path appiattimento LTI, segnali vettoriali (Mux/Demux) | v0.1 |
| **v0.3 — Discreto, ibrido e ciò che v0.2 ha rimandato** | Fast-path appiattimento LTI via operatori cryspace, segnali vettoriali (Mux/Demux) e sottosistemi multi-porta, blocchi discreti (UnitDelay, DiscreteSS, ZOH) usando `sample(dt)`, multi-rate semplice (dt multipli interi del base), blocchi condizionali **Switch/If** via `EEEval::CondParser` | v0.2 |
| **v0.4 — Avanzato** | Blocchi Observer (Kalman/Luenberger già in cryspace), solver adattivo RK45, risoluzione iterativa loop algebrici, **loader YAML del grafo** (reso completo dalle espressioni eeeval: anche sorgenti e Fn custom serializzabili) | v0.3 |

---

## 9. Rischi e decisioni aperte

1. **Loop algebrici** — la decisione di *rifiutarli* in v0.1 è la più importante per la semplicità del motore. Va documentata bene con la soluzione standard (inserire dinamica veloce o un UnitDelay).
2. **PIDController è discreto per natura** (`update(error, dt)`): dentro un motore RK4 continuo va trattato come blocco a passo campionato = `dt` del solver. Corretto per v0.1 se documentato; la forma continua del PID come StateSpace + derivata filtrata è l'alternativa pulita per il fast-path LTI.
3. **Prestazioni per-blocco** — con segnali scalari e dispatch virtuale per blocco, il costo per passo è dominato dall'overhead, non dalla matematica. Accettabile per v0.1; il fast-path LTI (v0.2) e l'uso di tensori pre-allocati (evitare allocazioni in `output`) sono le due leve note.
4. **`with yield` e scoping** — dentro il blocco DSL i metodi dell'oggetto chiamante non sono direttamente visibili; è un comportamento noto dei DSL Crystal da segnalare nella documentazione (workaround: variabili locali catturate dalla closure).
5. **API stability di cryspace** — CrySim diventa il primo *consumer* strutturale di cryspace; conviene definire presto quali API cryspace sono considerate contratto pubblico (StateSpace, Solver, TransferFunction, PIDController).

---

## 10. Conclusione

La combinazione cryspace + num.cr copre già integralmente la matematica del set minimo: solver ODE Proc-based riusabili come motore, LTI completi, PID con anti-windup, plotting HTML e persistenza Arrow. CrySim v0.1 è quindi un progetto a **rischio tecnico basso e valore concentrato nel design del DSL**: grafo a blocchi + `with yield` + macro di registrazione + validazioni con messaggi chiari. Il percorso incrementale (co-simulazione generale prima, appiattimento LTI come ottimizzazione poi) permette di avere esempi funzionanti end-to-end molto presto senza precludere le prestazioni.
