# Architecture Overview

## Task Planning

Gets all the destinations and returns an execution order.

## Path Planning

Needs to tell the robot in what direction to go. We can give the robot some constant speed.

## Navigation

Receive commands from the path planning layer. If obstacle detected (wall), trigger wall algorithm, until we are safe to continue.

## How to organize navigation? 

We need a priority:
1. Move forward
2. If obstacle detected, trigger wall-following
3. 