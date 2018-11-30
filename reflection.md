# CarND-Path-Planning-Project

## Software Architecture Overview

1. See diagram below

![software architecture overview](report/architecture.png)

## Module breakdown - FSM

This is the finite state machine responsible for high level decision making. It has 6 states - ready, notReady, keepLane, laneChangeLeft, laneChangeRight, final. Final is the only accepting state. At every call of run(), the fsm performs the following steps in order:

1. updates localization and environment information, including maximum safe speed for every lane
2. checks with PathPlanner if we are still in transit to the next state, if yes updates currentState to nextState
3. if still in transit, we do not calculate next state. Instead we call PathPlanner.extendPath() which takes the last 2 points of the current trajectory and simply extends it with same velocity and 0 acceleration.
4. if not in transit, we run through all the possible transitions and calculate the cost with respect to each possible transition. The 3 cost functions are:
 - goalCost : distance of targetLane from goal
 - inefficiencyCost : distance of maximum speed in targetLane from speed limit
 - safetyCost : checks for s distance to car in front and behind for targetLane
5. We take the target configuration with lowest cost, which may be keepLane, laneChangeLeft or laneChangeRight. Depending on the target configuration, we ask the PathPlanner to:
 - if keepLane and targetSpeed is different from currentSpeed by more than 5 mph, we ask the PathPlanner to generate a new trajectory, else we simply extend the current trajectory with PathPlanner.extendPath()
 - if target configuration is laneChangeLeft or laneChangeRight, we call PathPlanner.generatePath()


Since FSM is a high level module, units are in miles per hour.

## Module breakdown - PathPlanner

The PathPlanner is a low level module responsible for planning and executing the high level decisions made by the FSM. It also takes in environment information and calculates collision trajectories for other cars, and factors that in when generating trajectories and weighting them.


Since PathPlanner is a low level module, units are in metres per second.

## Event loop - how to generate Jerk Minimizing Trajectories



## Example trajectories

1. Lane change left or right example. ![lane change right/left example](report/laneChangeRight.png)
2. Speed up from 0 to 47 miles per hour. ![speed up from 0 to top speed 47mph](report/speedUp.png)
3. Slow down because ego car is approaching car in front. ![slow down because approaching car in front](report/slowDown.png)

## Challenges

## Solutions

## Future improvements
