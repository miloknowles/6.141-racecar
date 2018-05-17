# Lab 5 - Particle Filter Localization

See Stellar for the lab 5 handout, and RangeLibc documentation.

## Setup
- An RViz configuration for debugging is in ```cfg/localization.rviz```.
- I moved the hardcoded motion model and sensor model params into ```cfg/params.yaml```.
- I added some tests in ```src/test.py```, feel free to add to those!

## Launching
```
roslaunch headless_simulator teleop.launch # Just run once and then shut down.
roslaunch lab5_localization full_simulation.launch # Starts up all necessary simulator stuff.
roslaunch lab5_localization localize.launch # Starts our node after loading params.
```

## Gotchas
```
export SCANNER_TYPE=none # If teleop complains.
```
