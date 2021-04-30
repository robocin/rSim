# rSim

Robot soccer simulator based on [FiraSim](https://github.com/fira-simurosot/FIRASim) and [GrSim](https://github.com/RoboCup-SSL/grSim).

Simulates games from LARC Very Small Size and RoboCup Small Size leagues.

## Install Requirements
Debian:
```shell
$apt-get update
$apt-get install cmake qt-default libode-dev
$pip install numpy
```

## Build and install Python package
```shell
$mkdir build
$cd build
$cmake ..
$make -j4
$cd ..
$pip install -e .
```

## Usage VSS simulator
```python
from robosim import SimulatorVSS

sim = SimulatorVSS(field_type=0, n_robots_blue=3, n_robots_yellow=3)
```


## Usage SSL simulator
```python
from robosim import SimulatorSSL

sim = SimulatorSSL(field_type=0, n_robots_blue=11, n_robots_yellow=11)
```

## Running with Docker
```shell
$docker build -t robosim .
$docker run -it robosim /bin/bash
```
