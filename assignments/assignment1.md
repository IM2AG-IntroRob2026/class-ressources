# Assignment 1: Turtle Square

## Objective
Implement a ROS2 node that moves the `turtlesim` turtle in a square pattern using an **Action** interface.

## Requirements

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

## Extra Credits
- **Documentation**: Clear `README.md` with build and run instructions.
- **Verification**: A test script or `pytest` suite that triggers the action and verifies completion.

## Evaluation
Automated tests will be run against your action server. Ensure your interface and topic names match the specification exactly.
- Action name: `/draw_square`
- Action type: `turtle_square_interfaces/action/DrawSquare`
- Subscription: `/turtle1/pose`
- Publication: `/turtle1/cmd_vel`
