# Path Planning
As part of the Self-driving nanodegree programm I have written a Path Planning Algorithm in C++. To test the algorithm a simulator was provided by udacity.

[//]: # (Image References)

[traffic_1]: ./res/traffic_1.png "Traffic 1"
[traffic_2]: ./res/traffic_2.png "Traffic 2"
[follow_1]: ./res/follow_1.png "Follow 1"
[follow_2]: ./res/follow_2.png "Follow 2"
[lane_change_1]: ./res/lane_change_1.png "Lane Change 1"
[lane_change_2]: ./res/lane_change_2.png "Lane Change 2"

## Goals
From project [README](https://github.com/udacity/CarND-Path-Planning-Project):

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Simulator

The car is able to navigate trafic,

![alt text][traffic_2]
![alt text][traffic_1]

follow cars and

![alt text][follow_1]
![alt text][follow_2]

make lane changes:

![alt text][lane_change_1]
![alt text][lane_change_2]

## Model Documentation

* *The car drives according to the speed limit*: 
  This was achieved by setting the highest target velocity for each lane to the speed limit ([code](https://github.com/i4guar/CarND-Path-Planning/blob/master/src/main.cpp#L109-L114))
  ```          
  // target speed for each lane (max possible speed)
  vector<double> target_speed_for_lanes;
          
  target_speed_for_lanes.push_back(SPEED_LIMIT);
  target_speed_for_lanes.push_back(SPEED_LIMIT);
  target_speed_for_lanes.push_back(SPEED_LIMIT);
  ```

* *Max Acceleration and Jerk are not Exceeded*: 
  This is done by using splines of [spline.h](https://kluge.in-chemnitz.de/opensource/spline/) ([relevant snippet in my code](https://github.com/i4guar/CarND-Path-Planning/blob/master/src/main.cpp#L239-L272)). The splines provide a smooth trajectory and the max acceleration prevents the car from braking too hard and picking up to much speed.
  ```          
  const double ACCELERATION = 1.5; // m/s^2

  if (abs(speed_diff) < ACCELERATION) {
    desired_velocity = target_speed;
  } else if (speed_diff > 0) {
    desired_velocity += ACCELERATION;
  } else {
    desired_velocity -= ACCELERATION;
  }
  ```
* *Car does not have collisions*:
  The car always has a safety distance to the car infront and slows down if necessary. Furthermore, it checks if the neighbouring lane is not blocked before changing lanes
  ([relevant snippet in my code](https://github.com/i4guar/CarND-Path-Planning/blob/master/src/main.cpp#L116-L146)).

* *The car stays in its lane, except for the time between changing lanes*: Frenet coordinates ensure that the vehicle targets the middle of the road. I wrote a helper function, which converts each lane center to the d of Frenet coordinates ([relevant snippet in my code](https://github.com/i4guar/CarND-Path-Planning/blob/master/src/helpers.h#L40-L45)).

* *The car is able to change lanes*:
  The car makes a lane change if another lane has a higher target speed than the current lane
    ([relevant snippet in my code](https://github.com/i4guar/CarND-Path-Planning/blob/master/src/main.cpp#L149-L156)). 

By matching these criteria the first criteria is fullfilled: *The car is able to drive at least 4.32 miles without incident*.
