# dronfest22_simple

This repository contains scripts and utilities for controlling Crazyflie drones, including reading and writing Lighthouse base station memory, generating trajectories, and managing a swarm of drones.

## Features

- **Swarm Control**: Manage multiple Crazyflie drones with the ability to charge, take off, and execute flight missions.
- **Trajectory Generation**: Generate and upload custom trajectories (e.g., spiral, figure-8) for drones.
- **Lighthouse Memory Management**: Read and write Lighthouse base station geometry and calibration data.
- **Battery Monitoring**: Monitor battery levels and ensure drones are charged before flight.

## Repository Structure

```
├── battery.py               # Battery monitoring and drone control
├── cload-all.sh             # Script to flash firmware to multiple drones
├── demo.py                  # Main script to run a demo mission with multiple drones
├── gen_traj.py              # Generate reference trajectories for drones
├── read_lighthouse_mem.py   # Read Lighthouse base station memory
├── swarm_charge.py          # Swarm control and mission management
├── write_lighthouse_mem.py  # Write Lighthouse base station memory
├── cache/                   # Cached data for Crazyflie communication
├── README.md                # Project documentation
├── .gitignore               # Git ignore rules
└── __pycache__/             # Python bytecode cache
```

## Requirements

- Python 3.8 or higher
- Crazyflie Python library (`cflib`)
- `numpy` for trajectory generation
- `gTTS` and `playsound` for text-to-speech notifications (optional)

Install dependencies using:

```sh
pip install -r requirements.txt
```

## Usage

### 1. Flash Firmware
Use the `cload-all.sh` script to flash firmware to multiple Crazyflie drones:

```sh
./cload-all.sh
```

### 2. Run a Demo Mission
Run the `demo.py` script to execute a predefined mission with multiple drones:

```sh
python3 demo.py
```

### 3. Generate Trajectories
Use `gen_traj.py` to create custom trajectories (e.g., spiral, figure-8):

```py
from gen_traj import ReferenceTrajectory

trajectory = ReferenceTrajectory(curve="spiral", ts=0.1, N=5000, space=[0.5, 0.5, 1.5])
```

### 4. Read/Write Lighthouse Memory
- Use `read_lighthouse_mem.py` to read Lighthouse base station geometry and calibration data.
- Use `write_lighthouse_mem.py` to write updated geometry and calibration data.

### 5. Swarm Control
Use `swarm_charge.py` to manage a swarm of drones, including charging, takeoff, and flight missions.

```py
from swarm_charge import SwarmCharge

uris = {'radio://0/100/2M/E7E7E7E701', 'radio://0/100/2M/E7E7E7E702'}
with SwarmCharge(uris) as swarm:
    swarm.demo_mission()
```

## Notes

- Update the URIs in the scripts to match your Crazyflie drones.
- Ensure the Lighthouse base stations are properly configured before running the scripts.

## License

This project is licensed under the GNU General Public License v3.0. You are free to use, modify, and distribute this software, provided that any modifications or derivative works are also licensed under the same terms. For more details, see the [LICENSE](LICENSE) file.

## Repository

This project is publicly mirrored on GitHub under the name [drone-charging-demo](https://github.com/yourusername/drone-charging-demo).