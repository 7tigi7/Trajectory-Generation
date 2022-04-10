# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.
[Spline header link](https://github.com/ttk592/spline/blob/master/src/spline.h)
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

##Rubric Checklist

## ✅ The car is able to drive at least 4.32 miles without incident.

![15 Miles](Docs/20min.jpg)

The code is not prepared for cases where someone is not paying attention and suddenly a lane changes in front of us and we go at high speed. If we have enough time, we will brake in time.

## ✅ The car drives according to the speed limit.
Our car doesn’t go faster than 46.5 miles per hour. We only accelerate if we are below this value or we do not have a slow-moving car in front of us.

## ✅ Max Acceleration and Jerk are not Exceeded.
The code takes care, it accelerates safely. No jerk errors came up.

## ✅ Car does not have collisions.
In the code, we monitor the distance from the car in front of us(main.cpp line 150-168), and when we approach it, we brake harder or change lane in time.(main.cpp line 172-224)

## ✅ The car stays in its lane, except for the time between changing lanes.
The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

## ✅  The car is able to change lanes
In the code, we use data from sensor fusion to assess which lanes are free and safe.(main.cpp 127-143)
For lane change i made an automaton that can go into three states.
![Lane Change](Docs/laneChange.gif)
![States](Docs/states.jpg)
The state change can be found at main.cpp line 176-201

## The trajectory generation 
The trajectories are calculated with spline based code depending on speed and passed waypoints.
The idea can be found during the project lessons. It came from Q&A video content.(main.cpp 229-321)
We need a shift and rotation, we define a spline and set points x,y to the spline.
We are setting up the next x,y values. Fist we collect the previous path coordinates,
then we filling up the rest of our path planner after filling it with previous points. The output will be always 50 points.
The coordinates are transformed back at main.cpp 313-314.
These steps are allowed to give us a continuous trajectory.

##Reflection

This was one of my favorite lesson all of them!
It was also very interesting to meet the A * algorithm and its extended version.
[A star algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
Trajectory generation was also an interesting topic so we learned that generation is (path + time scaling) the car reaches the sequence of points in a given time,
 the trajectory should be sufficiently smooth and respect torques, limits, accelerations and velocities.
[Trajectory Generation Topics](https://www.sciencedirect.com/topics/computer-science/trajectory-generation)
I am confident that with the experience and knowledge I have gained, I will be able to create a much more complex and sophisticated project based on this project. All thanks to Udacity!
