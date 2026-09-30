# Diagnosi geometrica della camera reale

30 settembre 2026. Acquisizione tramite `ssh limo` sul Jetson `nano`, con
`custom_start/limo_real.launch.py` e pipeline CV già in esecuzione.
Ipotesi fornita dall'utente: pavimento piano, pareti sul fondo.
Nessun comando di movimento, riavvio o modifica dei parametri attivi.

## Configurazione effettivamente attiva

| Voce | Valore osservato |
| --- | --- |
| RGB | `/rgb/image_raw`, 640 × 480, `camera_color_optical_frame` |
| Profondità | `/depth_camera/depth/image_raw`, 640 × 400, `16UC1` in mm |
| Frame profondità | `camera_depth_optical_frame` |
| Registrazione depth → RGB | `depth_registration: false` |
| Sincronizzazione richiesta al driver | `color_depth_synchronization: true` |
| Profondità corretta | 320 × 120, `32FC1` in metri, frame della depth |
| Piano configurato nella correzione | `z = -0.18 m` in `camera_link` |
| TF base → camera | traslazione `(0.10, 0, 0.065) m`, rotazione identità |

Il parametro **attivo** `plane_height_m: -0.18` differisce dal profilo locale
`cv_real.yaml`, che contiene `.nan`. Il codice remoto di `depth_correction`,
`visual_ptcld` e `RayCache` è stato letto per verificare i comportamenti descritti.

Calibrazioni ricevute da CameraInfo:

| Sensore | fx = fy [pixel] | cx [pixel] | cy [pixel] |
| --- | ---: | ---: | ---: |
| Depth | 476.9272 | 324.7290 | 203.6672 |
| RGB | 456.1652 | 327.3780 | 241.6266 |

Entrambi pubblicano coefficienti di distorsione nulli. La TF depth → RGB
contiene una traslazione di circa 3.01 cm e una piccola rotazione: i due
sensori non condividono il medesimo centro ottico.

## Dati e metodo

Un nodo diagnostico ha ascoltato i topic per 22 secondi, conservando 12 immagini
per ciascuno dei tre flussi, CameraInfo, TF e 100 campioni di odometria.
Questi campioni odometrici mostrano posizione invariata e una variazione di yaw
di circa 0.005 gradi. Non costituiscono una misura esterna del movimento.
Le immagini dei tre flussi non sono tutte simultanee; per verificare l'azione
del correttore sono state usate soltanto le **due coppie con timestamp identico**.

La depth grezza è stata retroproiettata con i suoi intrinseci e trasformata
in `camera_link`. Il pavimento è stato stimato con RANSAC e raffinamento SVD,
usando i punti validi nella porzione inferiore e limitando le normali candidate
a piani prossimi all'orizzontale. Le pareti non sono usate come pavimento.
La stima è stata controllata anche sui dodici fotogrammi separatamente.

## Risultati

### Il piano è a circa 18 cm, ma non è orizzontale in camera_link

Equazione stimata dalla mediana temporale, con coordinate in metri:

```text
0.0341881 x + 0.0111858 y + 0.9993528 z + 0.1796407 = 0
```

L'altezza perpendicolare è 17.96 cm e l'inclinazione della normale rispetto
all'asse Z della camera è 2.06 gradi. Nei singoli fotogrammi: altezza
17.94–18.04 cm, inclinazione 1.93–2.00 gradi, residuo RMS circa 3.9–4.4 mm
sui punti selezionati del pavimento. Il piano della mediana ha il 99.0% di
inlier tra i candidati con soglia 8 mm.

Sotto l'ipotesi di pavimento piano e robot appoggiato correttamente, questo
indica un'inclinazione relativa della camera non rappresentata dall'attuale
modello orizzontale. Non è una calibrazione completa della TF: da questo piano
non si determinano yaw né traslazione orizzontale, e restano possibili errori
sistematici di profondità/intrinseci. La parte di pavimento con depth valida è
concentrata soprattutto a destra; l'estensione del piano ai buchi dipende
dall'ipotesi di planarità fornita dall'utente.

### Il correttore riempie molti buchi usando il piano sbagliato

Nelle due coppie con timestamp identico, il 70.11% e il 70.40% dei pixel del
ritaglio è riempito. I valori grezzi validi restano invariati entro 0.5 micrometri
(arrotondamento float). I valori inseriti coincidono con la LUT del piano
`z = -0.18`, entro 0.1 micrometri. È quindi verificata l'origine sintetica
di gran parte della profondità usata in questa scena.

Nella mediana dei dodici frame, il 67.27% del crop non ha alcuna misura valida.
Questo valore differisce dalla percentuale per singolo frame perché la mediana
può recuperare un pixel valido in uno solo dei frame.

Esempi nella colonna 160 del crop **depth**; confronto tra LUT attuale e
intersezione dello stesso raggio con il piano misurato:

| Riga | LUT attuale [m] | Piano misurato [m] |
| --- | ---: | ---: |
| 60 | 0.888 | 1.068 |
| 80 | 0.660 | 0.754 |
| 100 | 0.526 | 0.583 |

Sono distanze lungo Z ottico, non distanze euclidee. Il piano è una previsione
nei pixel senza ritorno, non una misura diretta di quei pixel.

Il passaggio a `plane_height_m: .nan` da solo non risolve: l'algoritmo attuale
stima soltanto una quota dalle righe inferiori, mantenendo la normale verticale.
Inoltre riempie qualsiasi buco compatibile con la LUT, senza distinguere un
buco sul pavimento da uno su una parete o un ostacolo.

### La profondità viene poi associata ai pixel RGB sbagliati

La depth corretta conserva la geometria ottica della camera depth, ma
`visual_ptcld` la indicizza con i pixel delle etichette RGB e usa i raggi RGB.
Il controllo verifica soltanto che entrambe le matrici siano 320 × 120.
Non viene applicata una registrazione geometrica tra i sensori.

Per i punti depth validi sotto 1.3 m nella metà inferiore, riproiettarli
correttamente nell'RGB anziché usare gli indici proporzionali comporta uno
spostamento mediano di circa +12 pixel orizzontali e -37 verticali, nelle
coordinate RGB originali. La differenza dipende da posizione e profondità.

Per confrontare il risultato complessivo, si può intersecare direttamente il
raggio **RGB** con il piano misurato e confrontarlo con la profondità assegnata
dalla pipeline attuale. Nei pixel centrali di pavimento selezionati:

| Riga nel crop RGB | Profondità assegnata [m] | Piano sul raggio RGB [m] | Errore XY stimato [cm] |
| --- | ---: | ---: | ---: |
| 60 | 0.888 | 0.802 | 8.6 |
| 80 | 0.660 | 0.577 | 8.4 |
| 100 | 0.526 | 0.450 | 7.5 |
| 119 | 0.440 | 0.373 | 6.8 |

Questa tabella include entrambi gli errori e non va confusa con il confronto
sul singolo raggio depth della tabella precedente. È una previsione geometrica
condizionata alla planarità e alla calibrazione pubblicata, non una misura
dello spostamento di uno stesso oggetto da due pose diverse.

## Correzione da implementare e validare

1. Stimare la normale e la distanza del pavimento dai ritorni validi,
   con esclusione delle pareti e verifica della copertura spaziale/inlier.
   La LUT deve usare l'intersezione raggio–piano completa, non la sola quota.
2. Portare profondità, etichette e calibrazione nello stesso sistema di pixel.
   Una soluzione verificabile è retroproiettare la depth grezza, trasformare
   depth → RGB e riproiettare nell'RGB con gestione delle occlusioni; solo dopo
   applicare crop/ridimensionamento coerenti. La registrazione hardware è una
   possibile alternativa, da verificare sul dispositivo e sulle risoluzioni.
3. Dove si usa il piano per completare la profondità nell'RGB, intersecare i
   raggi RGB con il piano trasformato nel relativo frame. Conservare la
   distinzione tra misure e valori sintetici; non assegnare automaticamente
   il pavimento a buchi su oggetti o pareti.
4. Validare la TF camera–robot e osservare gli stessi bordi da più pose ferme.
   Il piano consente di verificare roll/pitch e altezza relativa al suolo,
   non da solo tutta la trasformazione camera–robot.

Non sono stati modificati il codice applicativo, le TF o i parametri sul robot.
Questa diagnosi dimostra incongruenze geometriche nella pipeline attiva; non
dimostra ancora che eliminandole scompaia tutto lo sfasamento nella mappa.

## Artefatti della sessione

Sul PC e sul robot: `/tmp/limo_camera_diagnosis_20260930/`.

- `frames.npz`: immagini acquisite.
- `metadata.json`: intestazioni, calibrazioni, TF e odometria.
- `analysis.json`: risultati del piano e confronti grezzo/corretto.
- `rgb.png`, `comparison.png`: immagine RGB e confronto diagnostico.
- `rgb_plane_comparison.json` (PC): confronto del risultato finale sui raggi RGB.

Script: `/tmp/limo_camera_capture.py` e `/tmp/limo_camera_analyze.py` sul PC
e sul robot. Il grafico usa una mediana temporale grezza e un frame corretto;
le verifiche numeriche di preservazione/riempimento usano invece timestamp
identici. Gli artefatti in `/tmp` vanno copiati prima della loro pulizia.


## Correzione applicata e verificata

Aggiornati `cv_package` locale e quello del Jetson. Il profilo reale abilita
la registrazione software della depth in RGB e la stima completa del piano;
la simulazione conserva il percorso precedente. Come richiesto dall'utente,
**tutti i buchi di profondità vengono completati dal piano, senza filtri di
classe**. Restano invalidi solo i raggi senza un'intersezione ammissibile entro
l'intervallo configurato. I valori registrati misurati non vengono sovrascritti.
Questa scelta sostituisce la raccomandazione iniziale di limitare il completamento
ai pixel riconosciuti come pavimento.

`visual_ptcld` rifiuta depth con frame diverso da RGB/CameraInfo e non usa più
il fallback raw non registrato nel profilo reale. La classe semantica viene
calcolata successivamente con le regole già presenti.

La prima verifica live ha individuato ricalibrazioni spurie: TF2 restituiva la
stessa trasformazione depth → RGB con differenze di circa 1e-16 nei coefficienti.
Il confronto geometrico ora usa tolleranza assoluta 1e-8 sulle trasformazioni;
le variazioni effettive di calibrazione/frame/dimensioni continuano a invalidare
il modello. Aggiunto un test specifico di regressione.

Validazione conclusa:

- 51 test passati: geometria, callback ROS reali/simulati e nuvola, in un dominio
  ROS isolato sul container del PC. Due warning NaN di un test preesistente.
- Replay dei dodici frame reali attraverso il callback corretto: mantenute tutte
  le misure registrate, completati circa 29 mila pixel, header nel frame RGB
  e timestamp depth preservato. RMS circa 5.6 mm sui punti selezionati del
  pavimento nell'ultimo frame; non è una misura di accuratezza esterna.
- Build del solo pacchetto `cv_package` riuscita sul Jetson.
- Riavviati solo `depth_correction` e `visual_ptcld`; sensori, SLAM e mapper
  non riavviati e mappe esistenti non cancellate.
- Calibrazione finale live: altezza circa 0.1762 m, inclinazione 2.27 gradi.
- Controllo live di 16 secondi: 74 immagini depth, 74 LUT, 153 nuvole non vuote,
  una sola LUT calibrata distinta. Depth e LUT in `camera_color_optical_frame`,
  nuvola in `base_link`; ultimo campione con 798 punti. Il pixel centrale di
  esempio vale circa 0.802 m.

Backup sul robot: `/tmp/limo_camera_backup_20260930_154435/`, con file originali,
manifest, parametri e comandi precedenti/attuali e log. I due processi sostitutivi
della sessione corrente sono collegati alla durata del launch di mapping tramite
un piccolo supervisore temporaneo; al prossimo avvio il launch carica normalmente
il profilo corretto installato. Nessun commit o push eseguito.

Dati successivi alla correzione sul robot:
`/tmp/limo_camera_after_fix_20260930/`. Copie locali dei risultati:
`/tmp/limo_camera_diagnosis_20260930/fix_validation.json` e
`/tmp/limo_camera_diagnosis_20260930/final_live_check.json`.

Resta la conferma fisica osservando lo stesso bordo da due pose diverse. Le
osservazioni errate già accumulate nella mappa precedente non sono state
rimosse automaticamente.
