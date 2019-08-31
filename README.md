Fault diagnosis and control for multirotors with sensor faults

Overview

This repository contains a fault diagnosis scheme and control strategy for multirotors with sensor faults induced by the ground effect. When there is no fault, a hierarchical control structure composed by a Proportional-Derivative (PD) and a Proportional-Integral (PI) controller is developed to guarantee that the multirotor can track a trajectory. The external PD controller receives feedback from a vision algorithm that uses a monocular on-board camera. The internal PI controller receives measurements from the altimeter. The sensor faults occur on the inner loop, and we counteract them in the outer loop. We design the fault detection unit as a logical process which depends on the weighted residual. We present two control strategies: the first combines the external PD controller and a function of the residual; the second treats the sensor fault as an actuator fault and compensates with a sliding mode action.

We provide a folder with the files for simulating the control strategies in Simulink. Also, we provide a ROS package with control strategies for a Parrot Bebop 2 quadcopter. Natively, the code was written for Kinetic Kame. The package has the following dependencies:

-EIGEN3

-angles

-control_toolbox

-bebop_msgs
