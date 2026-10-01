# Configurazione dei launch di custom_start

I valori predefiniti si modificano in questi file:

- [config/limo_real.yaml](config/limo_real.yaml): porta seriale, lidar,
  camera, posizione della camera, scale di sterzo e RViz; la sezione
  `camera_driver` contiene le opzioni del driver Astra.
- [config/limo_circuit.yaml](config/limo_circuit.yaml): mondo Gazebo,
  interfaccia grafica, clock simulato, posizione della camera e posa iniziale.
- [config/ekf.yaml](config/ekf.yaml): parametri EKF condivisi.
- [config/ekf_real.yaml](config/ekf_real.yaml): override EKF del solo robot
  fisico, applicati dopo quelli condivisi.

I valori di `launch` possono ancora essere modificati per un singolo avvio:

```bash
ros2 launch custom_start limo_real.launch.py use_camera:=false
ros2 launch custom_start limo_circuit.launch.py gui:=false
```

Un argomento CLI ha precedenza sul valore nel profilo YAML. Per usare un
profilo alternativo, copiarne uno e passare il suo percorso assoluto:

```bash
ros2 launch custom_start limo_real.launch.py config_file:=/percorso/limo_real.yaml
ros2 launch custom_start limo_circuit.launch.py config_file:=/percorso/limo_circuit.yaml
```

Nel profilo della simulazione, `world` può essere un percorso assoluto o
relativo alla directory `share/custom_start` installata. Le rotazioni della
camera e la posa iniziale usano radianti; le distanze usano metri. Il
publisher TF di Foxy riceve gli angoli nell'ordine yaw, pitch, roll.

I launch leggono i profili dalla directory installata del pacchetto. Dopo
una modifica ai YAML nel sorgente, ricompilare `custom_start` e ricaricare
l'ambiente, oppure usare `colcon build --symlink-install` per avere i file
di configurazione collegati al sorgente.
