# void

Sistema di inseguimento autonomo di un target (nuotatore) con camera + YOLOv8, sonar e filtro di Kalman 1D (fusione 50/50), testato in simulazione VRX/Gazebo (`sydney_regatta`) su WAM-V e pensato per una futura barca leggera (~3 kg) con thruster ApisQueen.

## File principali
- `start_boat.sh`: avvia Gazebo/VRX, spawna il nuotatore a più waypoint e configura i bridge ROS 2.
- `boat_tracker.py`: rilevamento persona con YOLOv8, stima distanza con filtro di Kalman 1D e controllo differenziale dei thruster.

## Avvio rapido
1. Terminale 1: `./start_boat.sh` (attendere ~10s)
2. Terminale 2: `python3 boat_tracker.py --ros-args -p use_sim_time:=true`
