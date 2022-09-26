# Walking in Narrow Spaces: Safety-critical Locomotion Control for Quadrupedal Robots with Duality-based Optimization
<p align="center">
  <img width="24.0%" src="docs/distance.gif">
  <img width="24.0%" src="docs/dcbf.gif">
  <img width="24.0%" src="docs/duality.gif">
  <img width="24.0%" src="docs/duality_dcbf.gif">
  <img width="48.0%" src="docs/exp_straight.png">
  <img width="48.0%" src="docs/exp_l.png">
  <img width="48.0%" src="docs/exp_v.png">
  <img width="48.0%" src="docs/exp_random.png">
</p>

## Introduction
Open source code of [Walking in Narrow Spaces: Safety-critical Locomotion Control for Quadrupedal Robots with Duality-based Optimization](TODO) without perceptive implementation part. 

## Getting Started

### Build

Check this awesome repo [legged_control](https://github.com/qiayuanliao/legged_control) and make sure you can successfully run the simulation. 

Then clone and build the cbf_controllers:

```
git clone git@github.com:HybridRobotics/Quadruped-NMPC-DCBF-Duality.git
catkin build cbf_controllers
```

### Test
Launch the Gazebo

```
roslaunch unitree_description empty_world.launch
```

Spwan the controller with exponential DCBF constraints

```
roslaunch cbf_controllers test_simple_dcbf.launch 
```

**OR** the controller with exponential DCBF duality constraints

```
roslaunch cbf_controllers test_simple_duality.launch 
```

Then set a 2D Nav Goal in the RViz for the robot standup, and type "trot" to set the gait. You can comand the robot by `move_base_simple/goal` or `cmd_vel` or use the `goal_sender.launch` to send a goal (recommanded).

```
roslaunch cbf_controllers goal_sender.launch
```

## Implementation Details
The OCS2 only supports continuous time formulation, and discrete in the solver internally. So the exponential DCBF duality constraints is formulate as:

$$
\begin{align}
    -\lambda_{\mathcal{R}}^T \mathbf{b}_{\mathcal{R}}(\mathbf{x}) -\lambda_{\mathcal{O}_i}^T\mathbf{b}_{\mathcal{O}_i} &\geq  \alpha + e^{-\gamma (t-t_0)} d_i^*(\mathbf{x}_0), \\
    A_{\mathcal{R}}^T(\mathbf{x}) \lambda_{\mathcal{R}} + A_{\mathcal{O}_i}^T \lambda_{\mathcal{O}_i} &= 0, \\
    \left\| A_{\mathcal{O}_i}^T \lambda_{\mathcal{O}_i} \right\|_2 &\leq 1, \\
    \lambda_{\mathcal{R}} \geq 0, \lambda_{\mathcal{O}_i} &\geq 0.
\end{align}
$$
