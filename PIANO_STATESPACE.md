# CrySim — Piano di sviluppo: definizione di sistemi via spazio degli stati (A, B, C, D)

> **Stato**: tutte e 5 le fasi sono implementate (2026-07-06). Guardia `dt` su `StateSpaceBlock`,
> overload `ss(id, sys:)` con dispatch automatico continuo/discreto, blocco `dss` con doppio
> buffer di stato (`@x`/`@x_next` + `commit_sample`, per non far trapelare x[k+1] nel passo che
> lo calcola — bug individuato e corretto durante l'implementazione), `Model#state_space_of`,
> `state_names:`/`output_names:` con log automatico per-stato (ruolo `:state`) e tooltip SVG
> leggibili, e (Fase 5) `ss` con matrici come espressioni eeeval + `params:` — un
> `Hash(Symbol, Float64)` o un `NamedTuple` letterale, lo stesso meccanismo di binding già usato
> da `use` per i sottosistemi parametrici, verso cui `params:` viene tipicamente inoltrato
> direttamente (`sub.ss ..., params: params`). Esempi commentati in `examples/09_piano_fase5.cr`.
> 23 spec verdi, vedi `spec/crysim_spec.cr`.

Piano verticale focalizzato sul blocco `ss` e sulla sua integrazione con `CrySpace::StateSpace`,
complementare al backlog generale in `PIANO_FEATURE.md` (in particolare le voci di area B
"UnitDelay, DiscreteSS, DiscreteTF" e di area D "linearizzazione"/"fast-path LTI", che qui
vengono dettagliate).

## Stato attuale

Il blocco esiste già (`src/crysim/builder.cr` metodo `ss`, `src/crysim/blocks/continuous.cr`
classe `StateSpaceBlock`):

```crystal
ss :plant, a: [[...]], b: [[...]], c: [[...]], d: [[...]], x0: [0.0, 0.0]
```

- Costruisce un `CrySpace::StateSpace.new(a, b, c, d)` e lo spacchetta in array Crystal piatti
  per il ciclo caldo per-step (`unpack`, `continuous.cr:71-77`).
- Eredita gratis la validazione dimensionale di cryspace (`StateSpace#validate_dimensions`,
  chiamata dal costruttore) — dimensioni incoerenti tra A/B/C/D falliscono già a costruzione.
- `direct_feedthrough?` è calcolato automaticamente da D ≠ 0 (`continuous.cr:67`), quindi
  l'ordinamento topologico e la diagnosi dei loop algebrici funzionano correttamente anche
  per i sistemi SS diretti.
- `TransferFcn` è implementato come sottoclasse di `StateSpaceBlock` (via `to_statespace`),
  quindi ogni miglioramento a `StateSpaceBlock` si propaga automaticamente anche ai blocchi TF.

## Gap identificato (da correggere per primo)

`StateSpaceBlock#derivative` (continuous.cr:100-107) implementa sempre `ẋ = Ax + Bu`,
**anche se l'oggetto `CrySpace::StateSpace` passato ha `dt` non nullo** (sistema discreto,
ad esempio il risultato di `sys.sample(0.01)`). In quel caso l'engine integrerebbe con RK4
una dinamica che andrebbe invece iterata come `x[k+1] = Ax[k] + Bu[k]` — un errore numerico
silenzioso, non un crash. **Fase 1 introduce un controllo esplicito che rifiuta a costruzione
un `dt` non nullo passato a `ss`/`StateSpaceBlock`, indirizzando verso il nuovo blocco discreto
della Fase 2.**

---

## Fase 1 — Consolidamento della base continua

Obiettivo: rendere `ss` sicuro e comodo quanto `tf` prima di aggiungere qualunque cosa nuova.

- **Guardia sul `dt`**: `StateSpaceBlock.new` solleva `ModelError` se `ss.dt` non è `nil`,
  con messaggio che rimanda al blocco discreto (`"sistema discreto passato a un blocco
  continuo: usa dss oppure ss ..., dt: ..."`).
- **Overload che accetta un `CrySpace::StateSpace` già pronto**:
  ```crystal
  ss :reduced, sys: original_sys.balred(order: 2)
  ss :obsv_form, sys: original_sys.to_observability_form
  ```
  oggi `ss` accetta solo matrici letterali; serve un secondo overload `ss(id, *, sys:, x0: nil)`
  che salta l'impacchettamento e riusa direttamente il risultato di una trasformazione cryspace
  (discretizzazione, riduzione, cambio di base) senza dover re-impacchettare a mano le matrici.
- **Messaggi d'errore più didattici** quando `x0.size` non combacia con l'ordine del sistema
  (oggi è un `ArgumentError` generico, va reso `ModelError` coerente con il resto del builder).

## Fase 2 — Blocco discreto gemello (`dss`)

- Nuova classe `Blocks::DiscreteStateSpaceBlock`: stesso output `y = Cx + Du`, ma l'avanzamento
  di stato è un **update campionato** (`sampled? true`, come già fanno `PID` e `Noise` —
  `x[k+1] = Ax[k] + Bu[k]` calcolato in `update_sample`, tenuto costante durante eventuali
  sotto-passi del solver). Riusa interamente il pattern già presente in `continuous.cr`.
- DSL:
  ```crystal
  dss :plant_d, a: [[...]], b: [[...]], c: [[...]], d: [[...]], dt: 0.01
  ```
- **Dispatch automatico**: l'overload `ss(id, *, sys:, x0: nil)` della Fase 1, quando riceve
  un `sys.dt` non nullo, costruisce direttamente un `DiscreteStateSpaceBlock` invece di
  sollevare l'errore — così lo stesso metodo `ss` funziona per entrambi i mondi e la scelta
  del blocco è guidata dal dato, non da una parola chiave diversa da ricordare.
- **Conversione continuo→discreto dentro il DSL**, riusando `sample(dt, method:)` di cryspace:
  ```crystal
  tf :plant_c, num: [1.0], den: [0.5, 1.0]
  ss :plant_d, sys: model.state_space_of(:plant_c).sample(0.01, method: :zoh)
  ```
  (dipende dalla Fase 3 per `state_space_of`).

## Fase 3 — Estrazione e round-trip verso cryspace

- `Model#state_space_of(name : Symbol) : CrySpace::StateSpace` — recupera l'oggetto cryspace
  sottostante a un blocco `ss`/`tf` del modello, per fare analisi diretta senza uscire dal
  modello: `model.state_space_of(:plant).poles`, `.bode_plot(...)`, `.lqr(...)`.
- Prerequisito diretto per due voci già in `PIANO_FEATURE.md`:
  - **Fast-path LTI** (area C): appiattire un sotto-diagramma serve prima estrarre le
    `StateSpace` dei blocchi coinvolti.
  - **Linearizzazione attorno a un punto operativo** (area D): il risultato della
    linearizzazione è esattamente un `CrySpace::StateSpace` da poter iniettare con `ss sys:`.

## Fase 4 — Ergonomia MIMO

- **Nomi di stati e uscite**: `ss :plant, a:, b:, c:, d:, state_names: [:pos, :vel],
  output_names: [:pos]` — usati per:
  - log automatico per-stato (ogni stato nominato diventa loggabile come un probe implicito,
    utile per ispezionare variabili interne di un sistema SS multi-stato senza uscite dedicate);
  - etichette leggibili nell'SVG al posto del generico `ẋ=Ax+Bu` quando il sistema ha pochi
    stati (es. mostrare `[pos, vel]` nel tooltip del blocco, coerente con `params_description`).
- Nessuna modifica al motore: è puro arricchimento di metadata sopra un meccanismo (log
  entries, tooltip) già esistente.

## Fase 5 — Costruzione parametrica ✅

- **Implementato**: `ss` accetta un overload con matrici come array di stringhe (espressioni
  eeeval, es. `"-k/m"`) più `params:` — un `Hash(Symbol, Float64)` (quello che un template
  `CrySim.subsystem` già riceve e può inoltrare così com'è: `sub.ss ..., params: params`), oppure
  un `NamedTuple` letterale per l'uso standalone fuori da un sottosistema
  (`params: {k: 40.0, m: 2.0, c: 0.5}`, esattamente come nell'esempio originale di questo piano).
  Ogni cella viene compilata e valutata una sola volta in fase di costruzione (le matrici di un
  sistema LTI sono statiche, a differenza di `signal`/`fn` che rivalutano `expr:` ad ogni passo).
  Un parametro non definito nell'espressione solleva un `ModelError` che nomina blocco ed
  espressione, non la `Exception` grezza di eeeval. `x0`/`state_names`/`output_names` restano
  letterali: solo le matrici A/B/C/D sono parametriche, perché sono l'unica cosa che cambia da
  un'istanza all'altra dello stesso template fisico. Vedi `examples/09_piano_fase5.cr` per un uso
  standalone e uno dentro un sottosistema parametrico instanziato due volte con valori diversi.

---

## Piano di test (per fase)

| Fase | Test |
|---|---|
| 1 | `ss` con matrici letterali produce la stessa uscita di `tf` per la stessa dinamica SISO (verifica incrociata già presente nello spirito delle spec attuali) |
| 1 | costruire `ss` con un `CrySpace::StateSpace` che ha `dt` non nullo solleva `ModelError` con messaggio esplicito |
| 1 | `ss sys:` con l'output di `to_observability_form`/`balred` produce un blocco equivalente (stessa risposta al gradino, entro tolleranza numerica) |
| 2 | `dss` con un sistema discreto noto (es. filtro IIR del prim'ordine `y[k] = a*y[k-1] + b*u[k]`) confrontato con la formula chiusa |
| 2 | `ss sys:` con `dt` non nullo dispatcha automaticamente a `DiscreteStateSpaceBlock` (nessun errore, risultato numerico coerente con `dss` esplicito) |
| 3 | `model.state_space_of(:plant).poles` restituisce gli stessi poli calcolabili con cryspace puro sullo stesso sistema costruito fuori dal modello |
| 4 | i nomi di stato compaiono come chiavi in `SimResult#signals` quando il blocco SS li dichiara |

## Sequenza consigliata

Fase 1 va fatta comunque a breve termine perché copre un difetto di correttezza silenzioso,
indipendentemente dal resto del piano. Fase 2 e Fase 3 sono la parte a valore più alto (sbloccano
due voci già pianificate in `PIANO_FEATURE.md`) e possono procedere insieme. Fase 4 è puro
arricchimento e può slittare senza rischio. Fase 5 resta bloccata dai sottosistemi parametrici
e non ha senso avviarla prima.
