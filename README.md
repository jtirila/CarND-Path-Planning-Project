# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
<br />
Project submission by J-M Tirilä

For a detailed description of the problem, see the corresponding Udacity repository: 
https://github.com/udacity/CarND-Path-Planning-Project
<br />

## Overview

In this project, the task was to build a path planner along with strategies for trajectory generation 
and selection to navigate a vehicle along a three lane highway with traffic. Various requirements 
concerning safe navigation and speed need to be met to pass the project.  
   
## My Solution 

In my solution I largely follow the strategy proposed in the project walkthrough video. Specifically, the 
trajectories are generated using splines along desired waypoints along the road, and 
a significant overlap between consecutive trajectory generation steps is utilized to ensure smoothness of the trajectory. 
Udacity staff also proposes an approximate heuristic for keeping the desired target speed along the reference trajectory.
The heuristic is employes as presented. 

In addition to the proposed approach, a set of variables is introduced to keep track of vehicle state 
and intended actions, and another set of variables and flags is used to keep track of the feasibility and efficiency 
of the various possible choices that can be made concerning the vehicle's future path. This collection of variables 
and flags constitute the state machine that controls the vehicle's possible actions. 

### Path Generation

Paths are generated using a spline based solution. All the waypoints remaining from the previous iteration are 
preserved, and new points are generated only to the extent that we always have 50 future points ready. With a typical
setup, this means that only a few new waypoints need to be generated for each step. 
This strategy, along with the usage of the spline, guarantee to an extent that the trajectories will be smooth, 
there actions are not too sudden so that jerk remains within reasonable limits.  

The spline based solution probably does not guarantee any kind of jerk minimizing behavior in a strict sense. However, 
the result is good and smooth enough. 

### The finite state machine

The solution employs a rough finite state machine in that it determines which of the future actions are safe to perform 
upon each iteration: 
1. Continue straight ahead with the current target speed
1. Change lanes to the left - also considers the possibility 
   of already being on the leftmost lane, in which case a change to the left 
   is not safe
   
1. A similar consideration about changing lanes to the right
1. As a fallback, if none of the above options is safe, remain in the same lane but slow down so 
   that no collision occurs  

Subsequently, trajectories are generated for each of the safe options and a cost function is used to pick the most 
efficient path.

Even though the finite state machine is not explicit in the solution, at every moment we choose one of the 4 states 
just described. 

### Checking safeness

The feasibility of any given path is a very complicated question. In this solution, it is handled using a rough 
heuristic and considering only the current position of the ego vehicle and those nearby, and the projected positions 
of the same vehicles at the end of the generated paths. The movement is assumed to be continuous enough so that   
this simplification is enough to prevent collisions from happening. 

The following safety consideration takes place for both current and 
projected positions. 

For each nearby vehicle, we check whether the vehicles are too close to each other in an absolute sense, 
and also whether they are likely to be too close to each other due to speed differences. 

The chosen thresholds for "being too close" are: 

* Another vehicle being closer than 20 meters ahead, or 8 metres behind, in our intended lane
* Another vehilcle being closer than 32 meters ahead, or 13 meters behind, in the intended lane, and the 
  car behind having a speed at least 15 miles per hour greater than the one ahead.  
  
If at least one of these two conditions is true for any of the observed vehicles, we flag the corresponding lane 
change as unsafe. Consequently, there will be not path generated for this option.  

### Generating the Trajectories

As described above, trajectories are generated using splines just as described in the project walkthrough video. 
The documentation for the spline generation may be found in the video so I will not detail it here. 

### Cost Function: Choosing the Best Trajectory

Once the trajectories have been generated, we need to pick the one that we 
will want to follow. The choice is made using a _cost function_. As only a small number of trajectories are generated
at each step, also the cost function is pretty simple. 

The most important considerations when building the cost functions were: 

 * To have the car advance as far as possible from the current location 
 * To avoid lane changes unless necessary 
 * Even when necessary to change lanes, try to consider the flow of traffic on each side so that an informed lane 
   change can be performed. 
   - In a naive version, we might change lanes into a blocked trajectory so that we soon need 
   to slow down and remain in a pocket. 
   - The third component of the cost function tries to mitigate this 
     by introducing a third, softer aspect of a lane being "blocked": even when a lane is considered safe, 
     we still flag it as having traffic ahead if there are vehicles in the lane further ahead. This flag 
     is then considered in the cost function, resulting in a penalty so that clear lanes are preferred over 
     ones having traffic.  


## Reflection

The performance of the car using the solution described above seems adequate for this exercise. Various improvements
could be considered, though. I'll describe some of them below. 

#### Mitigating the Hard Coded Safeness Thresholds by Using a More Sophisticated Measeure of Closeness 

In the current solution, the feasibility of the possible actions is determined using hard limits on whether something
can be done. This applies also to the choice of lanes in the presence of traffic that is not immediately blocking, but 
may slow the car down in future steps.  

In an alternative solution, one could construct a more continuous measure of blockedness of a lane. Instead of hard 
limits, we could construct a function that penalizes lanes really hard if there are cars very close to the ego 
vehicle in said lane, or ones approaching from behind with a great speed difference. Vehicles further ahead would 
result in greater penalties if travelling slowly, and the penalty would be reduced with the speed of the ego
vehicle's target speed approaching and the observed vehicle's speed from above. An observed vehicle travelling ahead 
but faster than the ego vehicle should probably result in a minimal penalty.  

With this continuous penalty approach, we would maybe be able to construct more trajectories and hence make more 
intelligent decisions concerning the future paths. However, the design of the cost function would then need more 
attention so that we would really end up not choosing dangerous paths. 

An immediate advantage of this continuous approach would be the elimination of any hard coded thresholds.   

#### Effects of Latency

Latency between observations of danger and the corresponding steering actions clearly manifests itself in situations 
where there are no lange change option and the ego vehicle needs to slow down in its lane, later on again adjusting
the speed upwards. This latency leads to a back-and-forth behavior where the ego vehicle alternates between 
slowing down and speeding up, as if always one step behind. A more advanced anticipatory model could be used when 
predicting the future locations of the vehicles so that this alternating behavior could be mitigated. 

#### More Sophisticated Strategies Around Jams 

When there are several cars ahead of the ego vehice, no attempt is made at estimating which lane will be available 
for driving first, or in a more complex situations, which lane choice will be more advantageous in terms of long 
run efficiency and complicated maneuvers around a bunch of vehicles. For a human driver, this happens quite naturally: 
one detects instinctively if an opening is not yet explicit but about to materialize so one can efficiently 
navigate through traffic. 

To come up with such a solution using a simple FSM, a smooth path generation strategy and simple speed adjustment 
operations would require quite a bit of twiddling with the logic, number of paths generated, and predictions 
concerning the future locations of other vehicles in relation to one another. No attempt is made in this solution 
into this direction. 

### More Accurate Predictions for the Movements of Other Vehicles

Even though the course material introduced different kinds of prediction techniques, such as the use of naive 
Gaussian classifiers, I ended up using a simplistic kinematic model for predicting vehicle behavior. That is, 
the positions of the other vehicles is just projected outwards in time using their current speed and longitudinal 
location. Hence, for example, no attempt is made at detecting ongoing lance changes of other vehicles. An obvious 
improvement to the current strategy would be to predict the state of the other vehicles in a manner somewhat similar
to choosing the state for the ego vehicle: detecting "keep lane", "change lanes to the left" e.g. states of
other vehicles and use those predictions to achieve a more accurate estimate of other vehicle's future locations.  
