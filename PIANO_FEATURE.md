# CrySim — Piano delle feature

Backlog esaustivo di tutto ciò che si può costruire sopra la v0.1 (implementata:
DSL `connect`, motore Euler/RK4, ~25 blocchi, probe inline, espressioni eeeval,
renderer SVG con ruolo/display label, SimResult con plot e CSV — vedi
`ANALISI_CRYSIM.md` e `README.md`), la v0.2 (sugar `>>`/`feedback`, sottosistemi
`subsystem`/`use` SISO, report HTML unificato con sparkline sui fili), la v0.3
(multi-rate semplice, `unit_delay`, `discretize`, `switch`, fast-path LTI scoped
alla catena semplice) e un giro successivo di robustezza/libreria (rilevamento
NaN/Inf con contesto, test di equivalenza contro cryspace, `lookup_table`,
`rate_limiter`, CI, audit dei buffer pre-allocati).

Legenda priorità: 🟢 alto valore/basso sforzo — candidato per il prossimo rilascio ·
🟡 valore medio o sforzo maggiore · 🔵 esplorativo/da validare con un caso d'uso reale
prima di investire.

---

## A. DSL e composizione

| Feature | Descrizione | Priorità |
|---|---|---|
| Sugar `>>` per catene ✅ | `step(:r) >> sum(:e, signs: "+-") >> pid(:c) >> tf(:g)` genera gli stessi `Wire` di `connect` — **implementato in v0.2** (`Block#>>`, backed da `ModelBuilder#wire_connect`) | 🟢 |
| `feedback from: to:` ✅ | sintassi dedicata per la retroazione, alternativa a `connect` esplicito — **implementato in v0.2** | 🟢 |
| Sottosistemi (`subsystem` + `use`) ✅ | blocchi riusabili e parametrici, inlined nel grafo padre (§4.3 dell'analisi) — **implementato in v0.2**, ma solo SISO (un inport, un outport per template): il caso multi-porta è rimandato, vedi sotto | 🟢 |
| Segnali vettoriali (Mux/Demux) | porte multi-canale, superamento del vincolo "1 filo = 1 scalare". **Rimandato a v0.4** (era già rimandato a v0.3, confermato invariato): non è un'aggiunta al DSL ma un cambio di architettura (ogni wire oggi porta un solo scalare; un bus richiede porte multi-valore in tutto il motore, non solo nel builder) | 🟡 |
| Sottosistemi multi-porta | superamento del limite SISO di v0.2 — bloccato dallo stesso lavoro di porte multi-canale di Mux/Demux, quindi **rimandato a v0.4** insieme ad esso | 🟡 |
| Subsystem gerarchico vero | sottosistema come nodo opaco (non inlined), con drill-down nel renderer | 🟡 |
| Blocchi condizionali (Switch/If/Compare) ✅ | **implementato in v0.3** (`switch`): `criteria:`/`threshold:` in Crystal puro per confronti relazionali (verificato che `EEEval::CondParser` non supporta affatto `>`/`<` — solo `==`/`!=`/`&&`/`\|\|`), più `condition:` via CondParser per selezione a valore esatto/modalità discreta | 🟡 |
| Loader YAML/JSON del modello | costruisce lo stesso grafo da file — le espressioni `expr:` eeeval lo rendono completo | 🟡 |
| Validazioni a compile-time via macro | arità dei parametri dei blocchi builtin controllata dal compilatore, non solo a runtime | 🔵 |
| CLI `crysim` (shard binary) | `crysim run model.cr`, `crysim diagram model.cr -o out.html` | 🔵 |

> **Nota di implementazione (v0.2)**: i sottosistemi hanno richiesto di cambiare
> l'identità interna di blocchi/wire da `Symbol` a `String` (`Wire#src`/`dst`,
> le chiavi di `SimResult#signals`, ...), perché Crystal non permette di costruire un
> `Symbol` a runtime (nessun `String#to_sym`) — necessario per generare nomi prefissati
> come `"m1.dynamics"`. Gli accessor pubblici (`result[:key]`, `model.state_space_of(:plant)`,
> ecc.) accettano ancora `Symbol` via overload, quindi il codice utente esistente non è
> stato impattato. Lo stesso passaggio ha scoperto un bug preesistente di v0.1: l'auto-naming
> di un blocco multi-output collegato a uno `scope` senza `as:` usava lo stesso pattern
> rotto di interpolazione di Symbol, producendo un simbolo-letterale illeggibile invece di
> un nome utile — ora si solleva un `ModelError` chiaro che richiede un `as:` esplicito,
> invece di tentare un auto-naming impossibile.

## B. Libreria di blocchi

| Feature | Base tecnica | Priorità |
|---|---|---|
| UnitDelay ✅, DiscreteSS ✅, DiscreteTF | **implementato in v0.3**: `unit_delay` (z⁻¹ auto-contenuto, sempre a rate base) e `dss`/`discretize` (quest'ultimo sugar su `sample(dt, method: :zoh/:tustin)` già in cryspace) | 🟢 |
| Multi-rate semplice (dt multipli interi) ✅ | **implementato in v0.3**: `dss` rispetta il proprio `dt` (rapporto intero col `dt` base, altrimenti errore), `pid` accetta `rate:`. Corretto nello stesso passaggio un bug latente: il motore aggiornava ogni blocco campionato ad ogni passo base, ignorando il rate dichiarato | 🟡 |
| Lookup Table 1D ✅ (2D non ancora) | **implementato**: `lookup_table` con interpolazione lineare, `extrapolate:` opzionale (clamp di default); nessuna dipendenza nuova. Il caso 2D resta backlog | 🟢 |
| Rate Limiter ✅ | **implementato**: `rate_limiter` con `rising_rate:`/`falling_rate:` (asimmetrico se dato, altrimenti simmetrico), stesso pattern a doppio buffer campionato di `unit_delay` | 🟢 |
| Relay / isteresi | logica a stati con soglie, ispirata a `Nonlinear` di cryspace | 🟡 |
| Transport Delay (ritardo puro) | approssimazione di Padé via `TransferFunction.pade` (già in cryspace) | 🟡 |
| Blocco Observer (Kalman/Luenberger/EKF/UKF) | wrapping diretto dei moduli già presenti in cryspace | 🟡 |
| Blocco LQR/LQG | wrapping di `lqr`/`lqg` di cryspace per controllo ottimo dentro il diagramma | 🟡 |
| Blocco SS discreto (`dss`), estrazione `state_space_of`, nomi di stato | piano dedicato in **`PIANO_STATESPACE.md`** — dettaglia e precede "UnitDelay/DiscreteSS" sopra | 🟢/🟡 |
| Rumori non gaussiani (uniforme, banda limitata) | estensione del blocco `Noise` esistente | 🔵 |
| Bus creator/selector | raggruppamento/estrazione di segnali multipli con nome | 🔵 |

## C. Motore di simulazione

| Feature | Descrizione | Priorità |
|---|---|---|
| Fast-path appiattimento LTI ✅ (caso semplice) | **implementato in v0.3, ma scoped**: `model.to_state_space`/`run_fast` funzionano solo per una singola sorgente → catena LTI in serie (senza diramazioni/feedback/blocchi non-LTI) → un solo sink, ridotta con l'operatore `*` di cryspace e simulata con `simulate` di cryspace. Verificato a precisione di macchina (~1e-15) contro il motore generale. Il caso generale (diramazioni, feedback, combinazioni parallele via `+`) resta **rimandato a v0.4**: richiede un algoritmo generale di riduzione sotto-grafo→StateSpace; farlo per i casi comuni ora, invece di tutti insieme, evita la trappola di correttezza che si rischiava provando a fare tutto in un colpo | 🟢 |
| `method: :midpoint` ✅ | **implementato**: RK2 esplicito a 2 stadi (t, t+dt/2), mai al bordo destro del passo — a differenza di RK4 non ha il blind spot sull'impulso di un passo esatto (converge al valore analitico al diminuire di `dt`, RK4 no a nessun `dt`) ed è più accurato di `:euler` ad ogni `dt`. Nato da un'indagine mirata sul bias 5/6 di RK4+impulso | 🟢 |
| Solver adattivo RK45 | `CrySpace::Solver.rk45` esiste già, va solo esposto come `method: :rk45` con passo variabile | 🟡 |
| Risoluzione iterativa dei loop algebrici | punto fisso/Newton invece del rifiuto secco attuale, per abilitare topologie oggi vietate | 🟡 |
| Rilevamento zero-crossing | eventi discreti (rimbalzi, cambi di stato) con localizzazione precisa dell'istante | 🔵 |
| Simulazioni batch / Monte Carlo | run multipli con seed diversi del blocco `Noise`, aggregazione statistica dei risultati | 🟡 |
| Generazione di codice specializzato | compilare il grafo in Crystal nativo senza dispatch virtuale per-blocco (oltre il fast-path LTI) | 🔵 |

## D. Analisi e post-processing

| Feature | Base tecnica | Priorità |
|---|---|---|
| Linearizzazione attorno a un punto operativo | Jacobiano numerico del diagramma non lineare → `StateSpace`, poi tutta l'analisi cryspace (poli, Bode, margini) | 🟡 |
| Ponte verso `ident` di cryspace | fit di una TransferFunction dai segnali loggati (es. dopo un `chirp`) | 🟡 |
| Analisi di sensitività / parameter sweep | wrapper che rilancia `model.run` variando un parametro, restituisce famiglia di `SimResult` | 🟡 |
| Solver del punto di equilibrio (trim) | trova x tale che ẋ=0 per un dato ingresso, utile come stato iniziale realistico | 🔵 |

## E. Visualizzazione / diagramma

| Feature | Descrizione | Priorità |
|---|---|---|
| Report HTML unificato ✅ | diagramma + sparkline dei segnali direttamente sui fili, un solo file (`result.report`/`model.report`) — **implementato in v0.2** | 🟢 |
| Rilevamento NaN/Inf con contesto ✅ | **implementato**: `CrySim::NonFiniteValueError` nomina blocco/porta/istante del primo valore non finito; `model.render_error(err, path)` evidenzia il blocco in rosso nel diagramma | 🟢 |
| Export PNG/PDF del diagramma | rasterizzazione dell'SVG per documentazione/slide | 🔵 |
| Drill-down sui sottosistemi | click su un blocco-sottosistema per espanderlo nel proprio diagramma | 🔵 |
| Diff strutturale tra due modelli | evidenzia blocchi/fili aggiunti, rimossi, modificati tra due versioni | 🔵 |
| Export Graphviz DOT (fallback) | alternativa di debug quando il renderer nativo non serve (§6.7 dell'analisi) | 🔵 |

## F. Persistenza e interoperabilità

| Feature | Base tecnica | Priorità |
|---|---|---|
| Sink `to_file` Feather/Parquet ✅ | **implementato**: `SimResult#to_feather`/`#to_parquet` in `src/crysim/arrow_io.cr`, richiesto solo sotto `-Darrow` (come fa num.cr stesso per il proprio backend Arrow — il build di default non lo vede nemmeno). Le classi reali (`Arrow::FeatherWriter`/`ParquetWriter`/`Table`/`Schema`/`DoubleArray`) vivono nella shard separata `eltony81/arrow.cr`, non in num.cr direttamente — verificato leggendone le sorgenti. Round-trip verificato empiricamente: file scritti da Crystal riletti con pyarrow, colonne e valori identici | 🟢 |
| Round-trip YAML completo | leggere/scrivere modello incluse sorgenti ed `Fn` custom come espressioni eeeval | 🟡 |

## G. Robustezza e diagnosi

| Feature | Descrizione | Priorità |
|---|---|---|
| Avvisi di rigidità (stiff system) | euristica sul passo RK4 rispetto alle costanti di tempo del modello | 🔵 |
| Validazione dimensionale opzionale | unità di misura sui segnali (es. rad, N·m) controllate a runtime | 🔵 — da validare, rischio di scope creep |

## H. Performance

| Feature | Descrizione | Priorità |
|---|---|---|
| Buffer pre-allocati end-to-end ✅ | **verificato**: audit di tutti i blocchi builtin, trovata e corretta un'allocazione per-step in `DiscreteStateSpaceBlock#update_sample` (sostituita con doppio buffer persistente scambiato via flag, non copiato) — nello stesso passaggio scoperto e corretto un bug di regressione (`commit_sample` scambiava i buffer anche nei passi non dovuti al multi-rate) | 🟢 |
| Benchmark dispatch-per-blocco vs fast-path LTI | quantifica il guadagno di §C per orientare la priorità del fast-path | 🟡 |

## I. Developer experience & tooling

| Feature | Descrizione | Priorità |
|---|---|---|
| `crysim init` | genera scaffold di progetto (shard.yml, esempio, spec) | 🔵 |
| Sito documentazione (mkdocs) | mirror dello stile già usato da num.cr | 🔵 |
| CI GitHub Actions ✅ | **implementato**: `.github/workflows/ci.yml`, checkout dei tre repo come sibling (rispetta le dipendenze `path:` locali di `shard.yml`), `crystal spec -Dopenblas` — verificato lo stesso comando in locale, non ancora osservato girare su GitHub | 🟢 |

## J. Testing & qualità

| Feature | Descrizione | Priorità |
|---|---|---|
| Test di equivalenza contro cryspace ✅ | **implementato**: confronto sistematico vs `step_response` (1e-6/1e-3 su 1°/2° ordine) e `impulse_response`. Ha scovato un bug reale non banale: con RK4 il blocco `impulse` converge a **5/6** del valore analitico, non al valore corretto, indipendentemente da `dt` — la valutazione `k4` cade esattamente sul bordo escluso dell'impulso. `method: :euler` non ha il problema (documentato in `Blocks::Impulse` e nel README) | 🟢 |
| Snapshot test del renderer SVG | rileva regressioni visive nel layout/glifi tra modifiche | 🟡 |
| Fuzzing del builder DSL | input malformati al `ModelBuilder`, verifica che ogni errore sia un `ModelError` con messaggio utile | 🔵 |

## K. Ecosistema

| Feature | Descrizione | Priorità |
|---|---|---|
| Pubblicazione shard ✅ | **implementato**: `shard.yml` ora dipende da `github: eltony81/cryspace` (v1.28.0) ed `eeeval` (v1.1.1, bumpata da 1.1.0 per includere 3 commit di sola documentazione già su `master` ma non ancora taggati) invece dei path locali. Repo `eltony81/crysim` creato e pushato su GitHub, versione bumpata a 0.4.0 (riflette le feature aggiunte oltre lo scope originale v0.3.0: robustezza, `:midpoint`, `lookup_table`, `rate_limiter`, sink Arrow), release v0.4.0 pubblicata | 🟢 |
| Storyboard di esempi ✅ (markdown) | **implementato**: `TUTORIAL.md`, 10 capitoli graduati dal primo modello fino a sottosistemi/multi-rate/fast-path/diagnostica, ogni snippet verificato contro la libreria reale. La galleria HTML interattiva in stile cryspace (MathJax, syntax highlight) resta backlog | 🟡 |
| Guida di migrazione da Simulink | tabella di corrispondenza blocchi Simulink ↔ blocchi CrySim | 🔵 |

---

## Sequenza consigliata

1. ✅ **Chiudere v0.2 come da roadmap dell'analisi**: `>>`/`feedback` sugar, sottosistemi `use`, report HTML unificato. Fatto.
2. ✅ **Poi la robustezza percepita**: rilevamento NaN/Inf con contesto e test di equivalenza contro cryspace. Fatto — ha anche scovato il bug del bias 5/6 su `impulse`+RK4.
3. **Ora l'espansione della libreria di blocchi**: `unit_delay`, `dss`/`discretize`, `switch`, `lookup_table`, `rate_limiter` fatti; restano Observer (Kalman/Luenberger), LQR/LQG, Relay/isteresi, Transport Delay — ha senso proseguire ora che il nucleo (motore + DSL + diagnosi) è stabile e verificato.
4. Le voci 🔵 restano nel backlog finché non emerge un caso d'uso concreto che le richiede: evitare di costruirle "perché si può".
5. Sink Feather/Parquet (F, 🟢) e CI (I, 🟢) — CI fatta; Feather resta da fare (richiede il flag `-Darrow`, non ancora verificato).
