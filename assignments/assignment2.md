# Assignment 2: Turtle drawing boundaries

## Objective

Using the architecture of your choice, implement a strategy for the turtle to draw the boundaries of its domain. The strategy should be interruptible: at any moment, a human operator must be able to interrupt the turtle and take manual control.  

## Requirements

### Expected behavior

- The turtle starts moving in a straight line
- When the turtle reaches the limit of the its domain, it follows it in one direction
- When following a boundary, the pen should be down, drawing
- When the turtle domain delimited by the pen is closed, the turtle should go back to its initial position
- At any moment, if the space bar is hit on the keyboard, the turtle should stop moving and listen to keyboard commands (teleop)
- When in manual mode, the turtle should resume its drawing boundaries behavior

### 1. Interface Specification

Create a custom action named `DrawSquare.action` in a package named `turtle_square_interfaces`.

**DrawSquare.action**:

```text
# Goal
float32 side_length
float32 speed
---
# Result
bool success
---
# Feedback
float32 remaining_distance
```

### 2. Implementation

- Create a package `turtle_square_controller`.
- Implement a node that provides the `DrawSquare` action server.
- The node must subscribe to `/turtle1/pose` to monitor progress.
- The node must publish to `/turtle1/cmd_vel` to move the turtle.
- Use **parameters** for:
  - `side_length` (default: 2.0)
  - `speed` (default: 1.0)

- The action should prioritize goal parameters over stored node parameters if both are provided.

### 3. Execution

The action should:

1. Move the turtle forward by `side_length`.
2. Rotate the turtle by 90 degrees.
3. Repeat 4 times.
4. Provide feedback on the remaining distance to complete the square.
5. Return `success: true` upon completion.

## Evaluation

Automated tests will be run against your action server. Ensure your interface and topic names match the specification exactly.

- Action name: `/draw_square`
- Action type: `turtle_square_interfaces/action/DrawSquare`
- Subscription: `/turtle1/pose`
- Publication: `/turtle1/cmd_vel`
