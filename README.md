# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the 
[releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with
other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be
provided the car's localization and sensor fusion data, there is also a sparse
map list of waypoints around the highway. The car should try to go as close as
possible to the 50 MPH speed limit, which means passing slower traffic when
possible, note that other cars will try to change lanes too. The car should
avoid hitting other cars at all cost as well as driving inside of the marked
road lanes at all times, unless going from one lane to another. The car should
be able to make one complete loop around the 6946m highway. Since the car is
trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop.
Also the car should not experience total acceleration over 10 m/s^2 and jerk
that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the
waypoint's map coordinate position, the s value is the distance along the road
to get to that waypoint in meters, the dx and dy values define the unit normal
vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the
road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID,
car's x position in map coordinates, car's y position in map coordinates, car's
x velocity in m/s, car's y velocity in m/s, car's s position in frenet
coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it
   recieves in the list every .02 seconds. The units for the (x,y) points are
in meters and the spacing of the points determines the speed of the car. The
vector going from a point to the next point in the list dictates the angle of
the car. Acceleration both in the tangential and normal directions is measured
along with the jerk, the rate of change of total Acceleration. The (x,y) point
paths that the planner recieves should not have a total acceleration that goes
over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is
BETA, these requirements might change. Also currently jerk is over a .02 second
interval, it would probably be better to average total acceleration over 1
second and measure jerk from that.

2. There will be some latency between the simulator running and the path
   planner returning a path, with optimized code usually its not very long
maybe just 1-3 time steps. During this delay the simulator will continue using
points that it was last given, because of this its a good idea to store the
last points you have used so you can have a smooth transition. previous_path_x,
and previous_path_y can be helpful for this transition since they show the last
points given to the simulator controller with the processed points already
removed. You would either return a path that extends this previous path or make
sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth
trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the
spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).


### Rubric points

#### Path Generation

Paths are generated using a spline based solution. All the waypoints remaining from the previous iteration are 
preserved, and new points are generated only to the extent that we always have 50 future points ready. This strategy, 
along with the usage of the spline, guarantee to an extent that the trajectories will be smooth, there actions are 
not too sudden so that jerk remains within reasonable limits.  

The spline based solution probably does not guarantee any kind of jerk minimizing behavior in a strict sense. However, 
the result is good and smooth enough. 

#### Determining Which Action are Safe


##### The finite state machine

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
efficient path. If none of the paths is safe, a fourth option is introduced: . 

Even though the finite state machine is not explicit in the solution, at every moment we choose one of the 4 states 
just described. 

##### Checking safeness

The feasibility of any given path is a very complicated question. In this solution, it is handled using a rough 
heuristic and considering only the current position of the ego vehicle and those nearby, and the projected positions 
of the same vehicles at the end of the generated paths. The movement is assumed to be continuous enough so that   
this simplification is enough to prevent collisions from happening. 

TODO: so the following takes place for both current and projected positions. 

Additionally, for each nearby vehicle, we check whether the vehicles are too close to each other in an absolute sense, 
and also whether they are likely to be too close to each other due to speed differences. 

The chosen thresholds for "being too close" are: 

* Another vehicle being closer than XX meters ahead, or YYY metres behind, in our intended lane
* Another vehilcle being closer than XXX meters ahead, or YYY meters behind, in the intended lane, and the 
  car behind having a speed at least MMM miles per hour greater than the one ahead.  

#### Generating the Trajectories

As described above, trajectories are generated using splines just as described in the project walkthrough video. 
The documentation for the spline generation may be found in the video so I will not detail it here. 

#### Choosing the Best Trajectory

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


### Reflection

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


> The car is able to drive at least 4.32 miles without incident..
> 
> The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.
> 
> The car drives according to the speed limit.
> 
> The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.
> 
> Max Acceleration and Jerk are not Exceeded.
> 
> The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

> Car does not have collisions.
> 
> The car must not come into contact with any of the other cars on the road.
> 
> The car stays in its lane, except for the time between changing lanes.
> 
> The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.
> 
> The car is able to change lanes
> 
> The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.
> 
> Reflection
> 
> CRITERIA
> MEETS SPECIFICATIONS
> There is a reflection on how to generate paths.
> 
> The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation".
