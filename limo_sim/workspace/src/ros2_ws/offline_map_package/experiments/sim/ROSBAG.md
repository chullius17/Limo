# Registrazione per gli esperimenti di mapping

Avvia Gazebo, SLAM e la pipeline visiva come nella normale simulazione. Poi,
da un terminale del computer:

```bash
docker exec -it limo_sim bash -lc '
source /opt/ros/foxy/setup.bash
python3 /workspace/src/ros2_ws/offline_map_package/experiments/sim/record_mapping_bag.py --name run_01
'
```

Lo script controlla la presenza di pointcloud semantica, scan, TF dinamiche e
statiche, `/clock` e `/map`. Registra anche `/odom` e `/odometry/filtered` se
pubblicati. Non introduce campionamento o una frequenza artificiale: registra
i messaggi ricevuti. QoS best-effort e carico del sistema possono comunque
causare perdite, da verificare dopo la registrazione.

Esegui un percorso rappresentativo, includendo alcune soste e ritorni sulle
stesse zone. Termina con **Ctrl+C** nello stesso terminale. La sessione si trova
in `experiments/sim/bags/run_01/`: `bag/` contiene il rosbag, mentre accanto sono
salvate le configurazioni, i parametri effettivi disponibili, le impostazioni
QoS e la coppia `ideal_map.pgm/.yaml`. Usa un nome nuovo per ogni sessione.

## Riproduzione per confrontare il solo mapper semantico

Ferma i produttori originali (Gazebo, SLAM e CV), poi avvia una nuova istanza
del mapper con `use_sim_time: true`, SLAM e CV disabilitati. Con il launch del
progetto, da un terminale ROS già configurato:

```bash
ros2 launch offline_map_package map.launch.py config_file:=/workspace/src/ros2_ws/offline_map_package/experiments/sim/bags/run_01/mapping_profile.yaml start_cv:=false start_slam:=false start_gui:=false start_rviz:=false use_sim_time:=true
```

In un secondo terminale ROS, avvia la riproduzione a velocità originale:

```bash
ros2 bag play /workspace/src/ros2_ws/offline_map_package/experiments/sim/bags/run_01/bag --rate 1.0 --qos-profile-overrides-path /workspace/src/ros2_ws/offline_map_package/experiments/sim/bags/run_01/qos.yaml
```

Su Foxy viene riprodotto il `/clock` registrato: non serve `--clock`, che non
è disponibile nel player installato. Non avviare una nuova SLAM durante questo
confronto: `/map` e `map -> odom` devono essere quelli registrati. Non usare
`--loop` sullo stesso mapper: ogni prova parte da una nuova istanza.

Prima dei confronti, verifica `ros2 bag info .../run_01/bag`, la presenza di
messaggi in tutti i topic richiesti e l'assenza di cloud scartate nel replay.
Per cambiare i parametri usa una copia del profilo della sessione, conservando
inalterati bag, riferimento ideale e configurazione originale.

La mappa ideale è nel mondo Gazebo; l'eventuale trasformazione verso il frame
SLAM `map` deve essere verificata e mantenuta uguale per tutte le prove.
