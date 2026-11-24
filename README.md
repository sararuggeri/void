## File principali
- `start_boat.sh`: avvia Gazebo, spawna il nuotatore , lo muove e configura i bridge ROS 2.
- `boat_tracker.py`: rilevamento persona con YOLOv8, stima distanza con filtro di Kalman 1D e controllo motori.
- `gps_tracker.py`: conversione delle coordinate GPS e registrazione della traiettoria.
- `my_boat.urdf`: file 3D della barca, con camere, GPS, IMU e sonar.

## Avvio rapido
1. Terminale 1: `./start_boat.sh`
2. Terminale 2: `python3 boat_tracker.py --ros-args -p use_sim_time:=true`

## Calibrazione stereocamera

Gli strumenti off-line per acquisizione, calibrazione, rettifica e test della
stereocamera sono mantenuti in [`stereo_calibration/`](stereo_calibration/).

Questa calibrazione non viene caricata automaticamente nel nodo ROS 2/Gazebo:
ogni parametro fisico dipende dalla camera reale e dalla sua risoluzione.
Nel modello URDF esistono attualmente due camere mono frontali da 640 x 360;
non usare i parametri di una camera fisica per il modello della simulazione
senza un passaggio esplicito di configurazione e validazione.
