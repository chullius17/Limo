# Gruppi di traiettorie di riferimento dell'MPC

Questo documento descrive le famiglie generate da `SamplingMpc::solve`,
comprese le tre partenze lente dedicate a fermo o quasi fermo.

Il riferimento globale arriva da `/limo/planning/path`. Il controllore genera
sequenze candidate di velocità longitudinale `v` e angolo di sterzo `delta`;
il modello Ackermann le trasforma in traiettorie previste del robot. Seleziona
la migliore ammissibile, applica soltanto il primo comando raggiungibile e
ripete la ricerca al ciclo successivo.

## Gruppi attivi nei profili sim e real

Entrambi i profili usano `structured_actions: true`, `action_variants: 3` e
`deduplicate: true`.

| Ordine di generazione | Gruppo | Identificatore | Proposte per ciclo |
| --- | --- | --- | ---: |
| 1 | Frenata e arresto | `Braking` | 1 |
| 2 | Continuazione della soluzione precedente | `Nominal` | 1 |
| 3 | Azioni strutturate in tre blocchi | `Structured` | 27 |
| 4 | Azioni monofase | `Monophase` | 27 |
| 5 | Partenze lente: destra massima, diritto, sinistra massima | `Startup` | 3 solo a bassa velocità |
| | Totale prima della deduplicazione | | **Fino a 59** |

### 1. Frenata e arresto — `Braking`

Il riferimento richiede `v = 0` e `delta = 0` per tutto l'orizzonte. Il modello
raggiunge l'arresto rispettando i limiti di decelerazione e di velocità dello
sterzo: il comando non viene azzerato istantaneamente dal generatore.

Questa candidata può vincere se le manovre in movimento sono peggiori o
inammissibili. Deve superare gli stessi controlli di collisione e di tubo.

### 2. Continuazione della soluzione precedente — `Nominal`

Quando esiste una soluzione valida precedente, la sua sequenza di riferimenti
viene traslata in avanti di uno o più passi temporali. In coda si ripete
l'ultimo riferimento disponibile. Questa candidata conserva una manovra già
iniziata e fornisce il warm start.

Al primo ciclo, oppure dopo un reset, viene invece mantenuto lo stato di
velocità e sterzo iniziale, riportato nei limiti ammessi. Da fermo può quindi
coincidere con la candidata di arresto. Una ricerca fallita cancella il warm
start.

### 3. Azioni strutturate in tre blocchi — `Structured`

Si combinano tre azioni longitudinali e tre azioni sullo sterzo:

| Azione longitudinale nel primo blocco | Diminuire lo sterzo | Mantenere lo sterzo | Aumentare lo sterzo |
| --- | --- | --- | --- |
| Frenare | Frenata + diminuzione | Frenata + mantenimento | Frenata + aumento |
| Mantenere la velocità | Mantenimento + diminuzione | Mantenimento + mantenimento | Mantenimento + aumento |
| Accelerare | Accelerazione + diminuzione | Accelerazione + mantenimento | Accelerazione + aumento |

Per ciascuna delle **9 combinazioni** vengono generate **3 varianti**, per un
totale di **27 proposte**. Diminuire o aumentare significa variare l'angolo
rispetto allo sterzo iniziale. Il riferimento viene limitato dall'angolo
massimo cinematicamente ammesso.

L'orizzonte dura **4 s** ed è diviso secondo `block_fractions: [0.25, 0.50, 0.25]`:

| Blocco | Intervallo | Riferimento di sterzo | Azione longitudinale |
| --- | --- | --- | --- |
| Primo | 0–1 s | `delta_1` | Accelerare, mantenere o frenare |
| Secondo | 1–3 s | `-delta_1` | Mantenere la velocità raggiunta |
| Terzo | 3–4 s | `delta_1` | Mantenere la velocità raggiunta |

Il terzo blocco torna al primo riferimento di sterzo, non necessariamente a
sterzo zero. Gli angoli sono riferimenti: il modello li raggiunge gradualmente
rispettando la velocità massima di sterzata.

Le tre intensità della variazione di sterzo sono circa un terzo, due terzi e
l'intera escursione parametrizzata, ciascuna moltiplicata per un fattore in
`[0.8, 1.0]` con l'attuale `action_noise: 0.20`. Il generatore usa seme fisso 42,
quindi non estrae perturbazioni nuove a ogni ciclo.

**Limite attuale:** il fattore un terzo/due terzi/intero viene applicato allo
sterzo, non all'accelerazione. L'accelerazione positiva è sempre tra l'80% e il
100% del limite, prima degli ulteriori vincoli dinamici. Da fermo queste
varianti possono quindi raggiungere tutte la stessa velocità massima, senza
offrire vere alternative di avanzamento lento.

### 4. Azioni monofase — `Monophase`

Per ciascuna delle 27 proposte a blocchi viene costruita una variante che
mantiene sull'intero orizzonte l'azione iniziale:

- Lo sterzo conserva il riferimento `delta_1`, senza controsterzata programmata.
- L'accelerazione o decelerazione del primo blocco continua fino al limite di
  velocità o all'arresto; l'azione di mantenimento conserva la velocità iniziale.
- La variante monofase e quella a blocchi condividono gli stessi riferimenti
  nel primo blocco.

Questo gruppo comprende accelerazioni in rettilineo o in curva, avanzamenti
a velocità mantenuta e frenate. Anche qui le tre varianti non forniscono
attualmente tre diverse velocità lente di partenza.

### 5. Partenze lente da fermo o quasi fermo — `Startup`

Il gruppo aggiunge esattamente tre riferimenti quando il modulo della velocità
iniziale del modello non supera `MPC.Startup.max_initial_speed`, configurato
a **0,05 m/s**. In modalità `OPEN_LOOP` questa velocità è quella dell'ultimo
comando, se disponibile; al primo ciclo viene dall'odometria.

| Manovra | Riferimento di sterzo sull'intero orizzonte |
| --- | --- |
| Destra massima | `-atan(wheelbase / min_turning_radius)` |
| Avanzamento diritto | `0` |
| Sinistra massima | `+atan(wheelbase / min_turning_radius)` |

Sono angoli assoluti, non incrementi rispetto allo sterzo iniziale. Lo sterzo
li raggiunge gradualmente rispettando `max_steering_rate` e gli altri limiti.
La velocità si avvicina a **0,03 m/s** con una rampa di **0,05 m/s²**, poi resta
costante nel resto dell'orizzonte. Se la velocità iniziale è leggermente
superiore al riferimento, la stessa rampa la riduce gradualmente.

I parametri `Startup.enabled`, `Startup.max_initial_speed`,
`Startup.target_speed` e `Startup.acceleration` sono definiti nei due YAML.
Le candidate passano attraverso lo stesso modello, la stessa deduplicazione,
gli stessi costi di smorzamento e gli stessi controlli di collisione e tubo.
Possono quindi essere scartate o perdere contro una manovra normale.

Il budget massimo passa da 56 a **59 proposte**: sopra la soglia iniziale le
tre partenze non vengono generate. La famiglia compare come **Partenza lenta**
nella telemetria e nell'anteprima, in viola.

## Deduplicazione e valutazione

Le proposte sono un budget di generazione, non traiettorie necessariamente
diverse. Il controllore elimina sia i riferimenti identici, prima della
simulazione, sia le sequenze identiche di velocità e sterzo effettivamente
raggiunte dal modello, prima della valutazione ambientale.

Da fermo, frenare o mantenere velocità zero può produrre la stessa traiettoria;
anche i limiti di velocità e sterzo possono far coincidere proposte diverse.
Traiettorie con lo stesso primo comando ma sviluppi futuri diversi restano
candidate distinte.

Una candidata può essere esclusa per costo di regolarizzazione già superiore
alla migliore soluzione, oppure interrompere in anticipo la somma dei costi
se non può più vincere. Il vincitore viene valutato completamente.

## Vincoli e costi comuni ai gruppi attivi

- **Dinamica e cinematica:** velocità, accelerazione, decelerazione, velocità
  dello sterzo, velocità e accelerazione angolare e raggio minimo di curvatura.
  I profili strutturati attuali consentono avanzamento e arresto.
- **Collisioni:** controllo di bordi e interno della footprint, anche nelle
  pose intermedie fra i passi della previsione. Collisioni con celle letali,
  celle sconosciute o uscita della footprint dalla costmap rendono la candidata
  inammissibile.
- **Tubo del percorso:** tutta la footprint deve essere nel tubo allo START
  e alla fine della previsione. Il raggio configurato è 0,60 m. Il vincolo non
  impone di restare nel tubo in ogni stato intermedio; i controlli anticollisione
  continuano ad applicarsi lungo tutta la manovra.
- **Costi ambientali e di percorso:** costmap locale con inflazione gaussiana,
  costo di distanza dal riferimento, critic di ostacolo e critic di percorso,
  inclusa la guida per l'aggiramento degli ostacoli quando attiva.
- **Costi terminali:** penalità per avanzamento inferiore al riferimento di
  1 m nell'orizzonte, con peso 2, e per errore di orientamento finale rispetto
  al percorso, con peso 1. Il metro è un obiettivo di costo, non un minimo
  obbligatorio di avanzamento.
- **Smorzamento:** penalità sulle variazioni di velocità e sterzo lungo
  l'orizzonte, sullo sterzo del primo comando e sulla variazione della velocità
  di sterzata rispetto al ciclo precedente. Questi termini valgono anche per
  le candidate strutturate e monofase.

In simulazione la previsione contiene 80 passi da 0,05 s; sul reale 40 passi
da 0,10 s. La scelta considera anche la traiettoria intermedia, le collisioni
e la regolarità dei comandi.

## Gruppi alternativi presenti nel codice, inattivi nei profili attuali

Con `structured_actions: false` e `Startup.enabled: false`, dopo `Braking` e `Nominal` vengono generati
i tre gruppi seguenti al posto di `Structured` e `Monophase`.

| Gruppo | Identificatore | Generazione |
| --- | --- | --- |
| Velocità e curvatura costanti | `ConstantCurvature` | Griglia di `velocity_samples × curvature_samples` riferimenti costanti sull'orizzonte; il modello applica comunque le rampe dinamiche. |
| Perturbazioni del nominale | `PerturbedNominal` | Variazioni gaussiane di velocità e sterzo della sequenza nominale, costanti per ciascun blocco temporale. |
| Esplorazione ampia | `BroadExploration` | Riferimenti di velocità e curvatura estratti nell'intervallo ammesso, indipendenti dal nominale e distinti per ciascun blocco. |

Le due ultime famiglie si dividono le proposte rimaste dopo frenata, nominale
e griglia a curvatura costante: una proposta residua ogni quattro usa
l'esplorazione ampia, le altre perturbano il nominale. Anche queste estrazioni
usano un seme fisso.

Per attivare questa modalità serve un `batch_size` compatibile con almeno
`2 + velocity_samples × curvature_samples`: cambiare soltanto il flag lasciando
il budget attuale di 59 non basta con le griglie dei profili distribuiti.

## Stato della procedura: SUCCESS

Dopo l'accettazione di START, la procedura termina in `SUCCESS` quando il centro
di `base_link` rimane dentro il cerchio finale per **2 secondi consecutivi**, con
velocità planare misurata non superiore a **0,01 m/s** e velocità angolare in
modulo non superiore a **0,05 rad/s**. Non è richiesto un orientamento finale
specifico per questa condizione.

Il raggio condiviso fra cerchio verde e riconoscimento dell'arrivo è
`path_executor.ros__parameters.goal_circle_radius` nel profilo sim o real:
il valore iniziale è **0,20 m**. `success_dwell_time` modifica la durata;
`stopped_linear_velocity` e `stopped_angular_velocity` modificano le soglie.
L'odometria arriva da `success_odom_topic`, inizialmente `/odometry/filtered`;
la posa del centro viene trasformata nel riferimento della traiettoria.

Il conteggio usa i timestamp ROS e riparte da zero se il robot si muove, esce
dal cerchio, mancano dati validi o TF, cambia il percorso, oppure viene richiesta
PAUSE o ABORT. Durante l'attesa di START o la pausa non si può entrare in SUCCESS.

Se Nav2 conclude prima, la procedura rimane `ACTIVE` per verificare l'arresto
misurato. Quando la condizione è soddisfatta, interrompe l'eventuale azione
FollowPath ancora attiva e conserva `SUCCESS` anche se arriva dopo un risultato
di cancellazione. La GUI torna al pulsante START. Un nuovo percorso, START o
ABORT azzera il risultato; la procedura non riparte automaticamente.

## Riferimenti nel repository

- Generazione, deduplicazione e regolarizzazione: [sampling_mpc.cpp](../../limo_dwb_critics/src/sampling_mpc.cpp).
- Adattatore ROS, vincolo terminale e selezione: [ackermann_mpc_controller.cpp](../../limo_dwb_critics/src/ackermann_mpc_controller.cpp).
- Parametri: [control_sim.yaml](../config/control_sim.yaml) e [control_real.yaml](../config/control_real.yaml).
