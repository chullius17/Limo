# LIMO application launch

## Simulazione: traiettoria sulla mappa ideale e controllo dai sensori

`launch/experiments/limo_app_ideal_sim.launch.py` avvia la mappa ideale del
circuito su `/map`, la localizzazione AMCL con laser e CV dell'esperimento
`online_map_ideal_sim`, `trajectory` e `control` con i profili di simulazione.
Il planner usa la mappa ideale; la costmap locale dell'MPC conserva laser,
griglia semantica CV con memoria e inflation, come nell'applicazione normale.
Il launch avvia una sola pipeline CV e il nodo `local_ctrl_map` con
`mapping_sim.yaml`. Tutti i nodi usano `/clock`; i grafici AMCL non vengono avviati.

Si apre anche la GUI di planning: in alto mostra la costmap globale usata da
SMAC, aggiornata dal topic configurato nel bridge (normalmente
`/global_costmap/costmap`), e in basso il log dei tentativi: posa di start e
arrivo, motivo del fallimento e successo finale. Il log riparte a ogni nuovo
goal e resta disponibile riaprendo la GUI. I due pannelli sono ridimensionabili.
La finestra mostra la costmap
elaborata del planner, distinta dalla mappa ideale `/map` e dalla costmap locale
dell'MPC.

Con Gazebo, robot, camera, `/scan` e odometria già attivi, compilare nel workspace ROS:

```bash
colcon build --packages-select online_map_package traj_package user_package --symlink-install
source install/setup.bash
ros2 launch user_package limo_app_ideal_sim.launch.py
```

Questo launch sostituisce l'applicazione simulata e il launch di localizzazione
ideale: non avviarli insieme. In RViz impostare la posa con **2D Pose Estimate**,
selezionare il goal con **2D Goal Pose** e avviare l'esecuzione con **START**
nell'interfaccia di controllo. AMCL confronta la CV con la mappa ideale e
`/scan` con la mappa statica dei muri perimetrali Gazebo, pubblicata su
`/limo/map_package/online/maps/laser_map`. Entrambi i pesi sono quelli di
`mapping_sim.yaml`: laser 2.0 e CV 1.0. Il laser alimenta anche gli ostacoli locali dell'MPC.
In RViz abilitare **Static Laser Map** per vedere il bordo di localizzazione.
Gazebo e il bringup del robot si avviano separatamente.

La mappa viene cercata automaticamente in
`offline_map_package/experiments/sim/ideal_map/ideal_map.yaml` nel sorgente.
La mappa laser affiancata è `ideal_laser_map.yaml`: risoluzione 0,05 m,
muri occupati, interno libero, esterno sconosciuto e nessuna inflation.
Deriva dalle collisioni di `custom_start/models/walls/model.sdf` e dalle pose
del mondo `limo_circuit_world.world`, comprese le pose degli include SDF.
Per rigenerarla dopo una modifica dei muri:

```bash
python3 src/ros2_ws/offline_map_package/experiments/sim/create_ideal_laser_map.py
```

Per mappe diverse o una pipeline CV già attiva:

```bash
ros2 launch user_package limo_app_ideal_sim.launch.py \
  map_yaml:=/percorso/ideal_map.yaml \
  laser_map_yaml:=/percorso/laser_map.yaml start_cv:=false
```

`start_rviz:=false`, `start_control_gui:=false`, `start_planning_gui:=false`
e `start_mpc_preview:=false`
disabilitano i rispettivi viewer. `planner_params_file`, `controller_params_file`
e `mpc_preview_params_file` consentono di usare YAML personalizzati; quelli
predefiniti sono i profili `sim` esistenti.

## Esperimenti EKF: confronto delle odometrie

Il launch unico `launch/experiments/ekf_experiment.launch.py` apre una finestra
con sei grafici aggiornati: traiettoria XY, yaw, velocità longitudinale, velocità
angolare, distanza fra le posizioni e differenza di yaw. Confronta `/odom` e
`/odometry/filtered` durante lo stesso movimento, senza pubblicare comandi o TF.

Compilare `user_package` nel workspace ROS e caricare l'installazione:

```bash
colcon build --packages-select user_package --symlink-install
source install/setup.bash
```

Con robot, sensori ed EKF già avviati, eseguire sul PC con ambiente ROS Foxy e
display grafico:

```bash
ros2 launch user_package ekf_experiment.launch.py profile:=real
```

Per Gazebo, lo stesso launch seleziona l'orologio di simulazione:

```bash
ros2 launch user_package ekf_experiment.launch.py profile:=sim
```

Il file rimane nella sottocartella `experiments` anche dopo l'installazione;
`ros2 launch` lo trova per nome. Il PC deve vedere gli stessi topic DDS del robot.
Il launch apre solo il confronto, quindi si può affiancare all'applicazione o
alla teleoperazione esistente. Per avviare anche il bringup e il suo EKF, quando
non sono già attivi, usare sul computer che ospita il robot o Gazebo:

```bash
ros2 launch user_package ekf_experiment.launch.py profile:=sim start_bringup:=true
ros2 launch user_package ekf_experiment.launch.py profile:=real start_bringup:=true
```

Sono due alternative. Il profilo reale avvia il bringup senza camera e RViz;
quello simulato usa il circuito Gazebo. Non vengono avviati planner, controller
o teleop. `bringup_config:=/percorso/profilo.yaml` seleziona un diverso YAML di
`custom_start`. Non avviare nuovamente il bringup se il robot è già attivo.

Opzioni del confronto:

```bash
ros2 launch user_package ekf_experiment.launch.py profile:=sim \
  raw_topic:=/odom filtered_topic:=/odometry/filtered \
  history_seconds:=300.0 max_samples:=40000 refresh_hz:=5.0 max_sync_gap:=0.2
```

Le differenze sono calcolate ai timestamp dell'odometria originale interpolando
la stima filtrata, senza estrapolazione e senza attraversare intervalli maggiori
di `max_sync_gap` secondi. Lo yaw usa la distanza angolare più breve. Sono
accettati solo campioni con `header.frame_id` e `child_frame_id` uguali e non
vuoti: non viene stimata o applicata alcuna trasformazione fra frame diversi.
La finestra mostra conteggi, età degli ultimi messaggi e RMS delle differenze
sui campioni confrontabili conservati. L'RMS non è un errore rispetto al vero.

**Pausa grafici** congela la vista, continuando ad acquisire. **Nuova prova**
svuota i dati locali senza cambiare la posa del robot. **Esporta CSV + PNG**
salva la vista corrente in una nuova cartella, con `raw.csv`, `filtered.csv`,
`differences.csv` e `odometry.png`. I CSV usano secondi ROS assoluti, metri,
radianti, m/s e rad/s; i grafici mostrano gli angoli in gradi. La memoria è
limitata sia dalla durata sia dal numero massimo di campioni: l'esportazione
contiene solo la finestra conservata, o quella congelata quando in pausa.
Per conservare l'esperimento completo, registrare anche un rosbag.
Un salto indietro maggiore di un secondo in uno stream azzera le due storie;
piccoli messaggi fuori ordine vengono scartati.

Il confronto funziona anche riproducendo un bag con `/clock`, impostando
`use_sim_time:=true` e lasciando `start_bringup:=false`.

Per dimostrare un miglioramento di accuratezza occorre un riferimento
indipendente. Sul LIMO reale `/odom` usa già lo yaw dell'IMU; il confronto non
è quindi fra sole ruote e ruote+IMU. Inoltre il plugin Ackermann standard Foxy
produce `/odom` dalla posa di Gazebo: per valutare realisticamente la fusione
servono ingressi sensoriali con errori, mantenendo separata la ground truth.

The application includes online localization/mapping, trajectory planning and
control. Start sensors first, then launch the application:

```bash
ros2 launch user_package limo_app_real.launch.py
```

CV starts through online mapping when `launch.start_cv` is true in the selected
`online_map_package/config/mapping_real.yaml` or `mapping_sim.yaml`. Both shipped
profiles enable it. `start_cv` defaults to empty, so the YAML setting is preserved.
To override the config for one run, use `start_cv:=true` or `start_cv:=false`.
If CV is already running separately, disable its application startup:

```bash
ros2 launch cv_package cv_real.launch.py
ros2 launch user_package limo_app_real.launch.py start_cv:=false
```

Explicit `start_cv` overrides are forwarded to online mapping, whose
`mapping_real.yaml` selects
`cv_real.yaml` and the waterfall detector. `limo_app_sim.launch.py` selects
`cv_sim.yaml` and the HSV detector. `limo_app.launch.py` also uses the simulation
profile, with a wall clock; changing the clock does not change the detector.

The application composition lives in `launch/limo_app.launch.py`; the real and
sim launches include it with their respective `profile` and `use_sim_time`
settings. The common launch also accepts those arguments directly, defaulting
to `profile:=sim use_sim_time:=false`.

An optional `cv_config:=/absolute/path/to/profile.yaml` overrides the selected
CV profile when starting CV. All consumers continue to receive the semantic
cloud on `/limo/cv_package/visual_ptcld/points`; the CV launch connects waterfall
labels to the point-cloud node. Detector runtime parameters are under `/lane_node`.

## Robot and desktop

On the physical LIMO, start the sensors and then the application:

```bash
ros2 launch custom_start limo_real.launch.py
ros2 launch user_package limo_app_real.launch.py
```

The real application runs online mapping/localization, the trajectory planner,
the RViz goal bridge, the Nav2 controller, the path executor and the velocity
mux on the robot. CV processing starts only when `launch.start_cv` in the
mapping YAML is true (or `start_cv:=true` is explicitly supplied).
The real application does not open RViz or the control GUI.
`start_control_gui:=true` is rejected for the real profile; use the desktop launch.
The simulation application retains its existing local RViz/control GUI behavior.

In the PC's ROS environment, start all graphical clients with:

```bash
ros2 launch user_package desktop_app.launch.py
```

This opens one RViz for mapping, planned paths and the local costmap, together
with the control GUI. The additional CV RViz is disabled by default, independently
of CV processing on the robot. Enable it explicitly with `start_cv_rviz:=true`.
Each RViz client runs in its own launch scope. The desktop launch starts no mapping, CV processing,
planner, controller, velocity mux, path executor or second goal bridge.
RViz publishes goals to the robot; the GUI uses the robot's existing control
services to start, pause, resume or abort a planned path.

Both simulation and desktop RViz profiles show two outlines around the robot:
**Trajectory Footprint** in dark blue uses `/global_costmap/published_footprint`,
and **Control Footprint** in orange uses `/local_costmap/published_footprint`.
These are the live planner/controller costmap footprints, including padding;
they follow the robot and reflect configuration changes without copying sizes
into RViz. They appear once their costmaps are active and localization is ready.
The control GUI contains the control buttons and status terminal, with no MPC
telemetry/image panel. Read MPC telemetry separately on `/limo/control/mpc_debug`.
The controller YAML's `FollowPath.MPC.Debug.preview_enabled` starts the MPC image
renderer by default in simulation and leaves it off on the real robot.
Override it for one run with `start_mpc_preview:=true` or `start_mpc_preview:=false`.
View `/limo/control/mpc_preview/image/compressed` separately with `rqt_image_view`.

The real RViz profiles show **Waterfall Lane Overlay** using a desktop
`image_transport` decoder. The robot sends JPEG on
`/limo/cv_package/detection/lane_waterfall_overlay/compressed`; the desktop
decodes it to a local raw Image topic for RViz Foxy, whose Image display does
not support the old `Transport Hint: compressed` configuration. Online, offline
and CV viewers use separate output topics under `/limo/desktop/`.
The decoder starts with RViz only when that profile enables the overlay.
RViz requires the `image_transport` and `compressed_image_transport` packages
on the PC. The overlay may take up to 30 processed camera frames to appear,
because its detector checks for debug subscribers at that interval.

Before selecting a goal, use RViz's **2D Pose Estimate** tool to set the robot's
actual position and heading on the map. AMCL must publish `map -> odom` before
the planner can finish activation. Until then, the GUI reports localization
unavailable. After localization is ready, select a **2D Goal Pose** to generate
a path; execution starts only when requested in the control GUI.

Before pressing START on the real robot, disconnect/release the AgileX phone
app or remote control and verify `/limo_status.control_mode == 1`. The driver
requests command mode only when the base node starts: if feedback stays at 2,
stop the existing sensor/base launch, disconnect the phone, then restart that
launch. Never run two base drivers. The GUI now refuses START while chassis
status is missing, stale, faulty or outside command mode, and cancels an active
path if command mode or feedback is lost. It never takes chassis ownership
automatically; another explicit START is needed after recovery. Raw
`motion_mode=2` is not used for this guard because this robot reports it even
in the mechanically converted Ackermann configuration. The real controller
profile uses 10 Hz with 128 MPC sequences on the Nano; simulated control
remains unchanged. Validate movement under supervision before unattended use.

Optional desktop switches:

```bash
ros2 launch user_package desktop_app.launch.py start_cv_rviz:=true
ros2 launch user_package desktop_app.launch.py start_mapping_rviz:=false
ros2 launch user_package desktop_app.launch.py start_control_gui:=false
```

`start_cv_rviz` only selects the desktop viewer; it does not change CV processing
on the robot. For a separately launched CV backend, use `start_cv:=false` on the
robot and `start_cv_rviz:=true` on the PC if its viewer is wanted.

Both launches accept `mapping_config:=/absolute/path/to/mapping.yaml`.
Use matching profiles on each machine; paths are local to that machine.
The desktop resolves `cv_config` from the mapping profile, with the same optional
`cv_config:=/absolute/path/to/cv.yaml` override as the robot. Relative CV profile
names are resolved in the installed `cv_package/config` directory.
All real nodes and desktop clients use the wall clock.

Build/install `user_package` in each ROS workspace before using the new launch.
PC and robot must share the same `ROS_DOMAIN_ID` and be discoverable over DDS;
otherwise RViz and the GUI cannot reach the robot.

## Planner and controller configuration

The real and sim app launches explicitly select the YAML files created for
trajectory and control, then forward them as `planner_params_file` and
`controller_params_file` to the common app launch. It passes those files to
`trajectory.launch.py` and `control.launch.py`, together with `robot_model`:

| Application | Trajectory YAML | Controller YAML |
| --- | --- | --- |
| `limo_app_real.launch.py` | `traj_real.yaml` | `control_real.yaml` |
| `limo_app_sim.launch.py` | `traj_sim.yaml` | `control_sim.yaml` |
| `limo_app.launch.py` | `traj_sim.yaml` | `control_sim.yaml` |

The legacy app keeps its wall clock while selecting simulation tuning.
Custom files can be supplied directly to the app:

```bash
ros2 launch user_package limo_app_real.launch.py \
  planner_params_file:=/absolute/path/to/planner.yaml \
  controller_params_file:=/absolute/path/to/control.yaml
```
