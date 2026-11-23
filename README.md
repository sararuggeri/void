## File principali
- `start_boat.sh`: avvia Gazebo/VRX, spawna il nuotatore a più waypoint e configura i bridge ROS 2.
- `boat_tracker.py`: rilevamento persona con YOLOv8, stima distanza con filtro di Kalman 1D e controllo motori.
- `my_boat.urdf`: file 3D barca, con aggiunta del sonar

## Avvio rapido
1. Terminale 1: `./start_boat.sh` (attendere ~10s)
2. Terminale 2: `python3 boat_tracker.py --ros-args -p use_sim_time:=true`
