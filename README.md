# Search & Rescue exploration simulator

:::info
This repository was created for the course: ME41125 Introduction to Engineering Research, given in Q4 of academic year 2022-2023. Author: Pelle Wiersma (4749804)
:::


## Purpose
The code provided in this repository aims to assess how well using audio as a heuristic for the locations of humans in an unknown environment, affects the **exploration time** until all humans in the environment are found.

## Structure
The code consists of 2 parts blabla

## Installation

1. First of all, clone this repository in your preferred location
2. Go to the terminal and head to the location where this repository was cloned
3. Create a virtual python environment and install the required packages by running the following commands.
```console
  conda create -n rescue_env
  conda activate rescue_env
  conda config --env --add channels conda-forge
  conda install numpy
  conda install pygame
  conda install -c anaconda scipy
  conda install -c anaconda pillow
```
:::success
Now all required packages are installed!
:::

## Running the code
:::info 
:bulb:
Parameters like which scenario to run and the properties of the robots can be set in `swarm_exploration.py` and `robot.py`, respectively.
:::
To run the simulation, enter the following command:
```console
  python3 swarm_exploration.py
```
A window should open which displays the simulation environment. The simulation will run the scenario 30 times. The resulting exploration times are displayed in the terminal. This simulation can take some time, depending on the performance of the computer used.

## License information
This code is released under the BSD license. See LICENSE.md for the details.

## Authors
Pelle Wiersma


 
