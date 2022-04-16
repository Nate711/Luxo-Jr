# Luxo Jr

## Test code
### `luxojr/luxojr.py`
* Commands the robot to jump using the main motor. Starts off by commanding a constant torque until a position threshold is met than uses PD control.

## Analysis code
### `log_analyze.ipynb`
* Analyzes the logs produced by `luxojr.py`. Plots position/velocity/torque etc vs time for various control strategies, settings, physical configs, etc

### `SEA_analysis.ipynb`
* Investigates the use of a spring between the actuator and leg of the robot in order to allow the actuator to put more energy in the jump by "running ahead" of the output link. 
* Think of it like attach a bungee cord to a boulder, you running ahead of the boulder holding the bungee cord, and waiting for the boulder to fling forwards.