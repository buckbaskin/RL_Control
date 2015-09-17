# RL_Control

## Overview
The code breaks down into 4 main components. 
The first is the agent that is responsible for the learning and converting sensor data into actionable Twist commands.
This is done using a Q Learing model that reduces state from a (wheeled-odometry, desired-odometry,) tuple-pair to 5 functions in the 2D implementation:
- Error along desired axis
- Error off desired axis
- Heading Error
- Linear Velocity Error
- Angular Velocity Error

Then using a transition model, the "optimal" Q can be found by iterating through Twist (just linear x, angular z in 2D case) to find the best resultng Q.
The transition model makes up the last 3 components (roughly ROS nodes) that make up the code.

The first of these 3 is the odometry (usually wheeled) state transition model. 
The node maintains its best estimate of a transition of the form f(state0, action) => state1.
The states are the data from odometry (wheeled or otherwise) that comprise the best estimate of the robot's actual position.
The model is broken into two components: the deterministic estimate of the robot's change in position, and an error term of the same dimensions.
The two of these combine to create a stochastic transition function that best represent's the robot's estimate of how it will move.
The function/model is used to estimate the new location of the robot for a given Twist.

The next node is the desired state transition model.
This node maintains its best estiamte of a transition f(des_state0) => des_state1.
The states are of the Odometry data type, and are published by a higher level planner node to "lead" the lower level controller by a carrot on a stick model.
The model is again broken into two components: the deterministic estimate and error term.
These two combine to create a general estimate of what the control is going to ask for in a future step.
The function/model is used to estimate the new location of the desired state for a given initial state.

Collectively, these two nodes represent the tuple-state transition function f(state, des_state, twist) => (state, des_state) that is used to estimate the values in the learning model.

The last node is auxiliary, and does not have a direct effect on the learning model (yet). 
Instead, what it does is track the actual states that occur, in one stochastic component.
This is used to check the deterministic transition model, because if the mean of the non-adjusted model starts to depart from the trantision model, then the model needs to be adjusted.

## Versions
- v0.1: Working implementation of odom_Transition (deterministic + error), desired_Transition
- v0.2: RL based off of those two components
- v0.3: Integrate 
- v1: Working RL w/ all 4 components. State space constrained to 2D wheeled robotics. Motion model is defined by an estimated deterministic transition function
- v2: State space in 3D.

## Reinforcement Learning
Coming Soon

## Location-Twist/Odom Transition Model
Coming Soon

## Desired State Transition Model
Coming Soon

## Independent/Audit Transition Model
Coming Soon
