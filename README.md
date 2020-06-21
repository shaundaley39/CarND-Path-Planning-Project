# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Path Planning - Problem Description

In past projects we programmed and tuned controllers for vehicle actuators, to closely follow a target trajectory.
In this project, we're treating that as a solved problem: the car will accurately follow any viable trajectory it is given.

The challenge for this project then, is to generate viable trajectories for a vehicle which avoid collisions with other vehicles, which comply with road rules (not driving outside of road lanes except to shift between them, not exceeding the speed limit) and which satisfy comfort constraints (total acceleration less than 10 ms<sup>-2</sup>, total jerk less than 10 ms<sup>-3</sup>, including centripetal acceleration and jerk).

This simulation provides a relatively simple environment: a three lane highway circuit with no intersections or traffic lights.
The environment is fully mapped: the circuit route is modelled by a series of waypoints (with tangent vectors), with constant lane widths. The speed limit is constant (50 miles per hour).

We assume localization to be a solved problem for the purpose of this project - we have fully accurate cartesian coordinates and orientation for our vehilce (provided by the simulator).

We assume the detection and speed estimates of nearby vehicles to be a solved problem (this is provided directly by the simulator).
Note however: other vehicles may (and do) change their speed erratically.
Other vehicles may shift lanes (without warning, and indeed the simulator doesn't inform us that this is in the process of happening until the vehicle has actually shifted lane).
This detection of "nearby" vehicles does not look very far, in particular for vehicles behind us when we are pulling out to overtake.
Fortunately vehicles behind us are ralely going fast and so they slow down and give way where necessary.
However, these limitations make responsible "Autobahn-style" driving impossible.
Lanes, lane positions and speeds of nearby vehicles are treated as a "solved" problem then for this project, but a solved problem with evident limitations.

## Implementation

#### Spline

To assist with generating trajectories along lanes, we generate an array of cubic spline models (one for each lane) using an appropriate [open source library](https://kluge.in-chemnitz.de/opensource/spline/spline.h).
This is handled in lines 66-109 in [main.cpp](src/main.cpp).

Note here: at first, a cubic spline was inferred directly using the 181 map waypoints for the track.
While this resulted in visually correct behaviour (the vehicle drove seamlessly in the centre of its lane around the entire track), the simulator failed the car as driving "out of lane".
The reason for this error was that the simulator (wrongly) judges the car as "out of lane" if it departs from a margin of the straight lines between successive waypoints.
This was "solved" by generating additional waypoints halfway between each of the provided "map" waypoints.
This slightly denser set of waypoints was sufficient to keep the car "in lane" according to the simulator's assessment, though in fact the vehicle now deviates more from the centre of its lane (judged visually).

#### Lane Model

For each planning cycle, the first interesting part of our code is the lane model update, seen in lines 190-215 in [main.cpp](src/main.cpp).

We consider the state of the world at the end of the trajectory that our car is already planned to follow.
For each lane, we approximate the "lane speed" as the speed of the next vehicle ahead of where our car will be (at the end of its prior trajectory), or as our target speed (46 miles per hour) if no such vehicle is detected.

For each lane, we also calculate an "s limit" - the s-coordinates of vehicles ahead of our vehicle, at the moment our vehicle ends its prior trajectory.
We also calculate (approximately) an "s free" position for each lane, which is the s-coordinate at which our car would have to begin decelerating to remain safely behind the next vehicle in that lane.
Finally, for each lane we determine whether the lane is safe to enter ("lane accessible"), considering not only the space ahead of our car but also whether there are any vehicles adjacent to our car, or indeed somewhat behind our car but faster moving.

#### Action Plan

In lines 223-232, we use our lane model to decide what to do next.
If there is a lane to the right of us which is accessible, and if there is no vehicle ahead of us in that lane for a substantial distance, we shift right.
(This isn't a stipulated requirement for the project, but there's never an excuse for bad driving. Nobody should ever hog the middle lane or left lane.)

Otherwise, if we are unconstrained in our current lane (the s position of our vehicle at the end of its previous trajectory is substantially less than the "s free" threshold), we can cruise at our target speed (46 miles per hour).

If we are close to becoming constrained or we are already constrained in our current lane (the s position of our vehicle, at the end of its previous trajectory, is close to or less than the "s free" threshold for the car's lane), we will move left to a faster lane if possible (if the lane is accessible, and if any vehicles obstructing that lane are further away from us).

Finally, if we are constrained in our current lane, we "track" the car ahead of us, aiming to maintain the same speed as it, keeping a little behind it so that we can drive more smoothly than the car in front and so we can break safely if necessary.

#### Trajectory Generation

From the above, we have an action plan of "track", "cruise" or "lane change" (with a target lane). In each case, we need to generate a trajectory that satisfies acceleration and jerk constraints. This is handled in lines 234-334 of [main.cpp](src/main.cpp).




---

# Practical Tips for Running this Project

### Simulator.
The Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

The "linux" binary was specifically built for an Ubuntu target, and is confirmed to run well on Ubuntu 16 LTS. With most recent Ubuntu distributions (or other distros such as Fedora), this won't work.
The version of Unity in which this simulator was written is no longer supported, does not run on any Linux distribution (although it builds for Ubuntu targets) and porting the simulator source code to more recent versions of Unity is non-trivial (many interfaces have been dropped or replaced and there have been changes to the environment mechanics). In short: if you only want to verify this project, it's probably best to temporarily set up a machine with an old Ubuntu distro to run the simulator.

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
5. Run the simulator (discussed above)

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

## Code Style

It would be ideal to closely follow [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
