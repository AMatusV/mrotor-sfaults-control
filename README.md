A Monocular SLAM-based Controller for Multirotors with Sensor Faults under Ground Effect

Overview

This repository contains a fault-tolerant scheme for a multirotor with altitude sensor faults caused by the ground effect. We assume a hierarchical control structure for trajectory tracking. The structure consists of an external PD controller and an internal PI controller. We consider that the sensor faults occur on the inner loop and counteract them in the outer loop. In a novel approach, we use a metric monocular SLAM algorithm for detecting internal faults. The fault diagnosis scheme is designed as a logical process which depends on the weighted residual. Furthermore, we propose two control strategies for fault mitigation. The first combines the external PD controller and a function of the residual. The second treats the sensor fault as an actuator fault and compensates with a sliding mode action. In either case, we utilize onboard sensors only.

We provide a folder with the files for simulating the control strategies in Simulink. Also, we provide a ROS package with control strategies for a Parrot Bebop 2 quadcopter. Natively, the code was written for Kinetic Kame. The package has the following dependencies:

-EIGEN3

-angles

-control_toolbox

-bebop_msgs
