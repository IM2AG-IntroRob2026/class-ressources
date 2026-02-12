# Lab3: Implementing a FSM in ROS2

## 1. Create a FSM package

Based on [this template](https://gist.github.com/adonze/41f1c54448fca2b6221667b5c55fd2cc), create a package and run it with turtlesim. 

Implement a launch file to start turtle and FSM. *

## 2. Extend with additional states and transitions

### Add events based on color sensing 

If the turtle sees red, it should back and turn.

Note: color can be added on the background with another turtle, or prior to executing 

### Add states based on color

E.g., double velocity when on red color.

### (More advanced) Add keyboard interruption

Combine with teleop. Switch between automatic and manual control at any time. 

## 3. Explore Yet Another State Machine (YASMIN) 

Can you reimplement the FSM with [YASMIN](https://github.com/uleroboticsgroup/yasmin) ? 






