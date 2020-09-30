# RoboSim

Robot soccer simulator based on FiraSim and GrSim.

Simulates games from Very Small Size and Small Size leagues of RoboCup.

## Install Requirements
```shell
$apt-get update
$apt-get install libboost-all-dev python-dev git cmake g++ gdb libgl1-mesa-dev libx11-dev libglu1-mesa-dev libode-dev
$pip install numpy
```

## Build and install Python package
```shell
$mkdir build
$cd build
$cmake ..
$make -j4
$cd ..
$pip install .
```

## Usage VSS simulator
```python
from robosim import SimulatorVSS

sim = SimulatorVSS(field_type=0, n_robots_blue=3, n_robots_yellow=3)
```

## Running with Docker
```shell
$docker build -t robosim .
$docker run -it robosim /bin/bash
```