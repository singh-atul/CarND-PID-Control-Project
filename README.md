# CarND-PID-Controller
Self-Driving Car Engineer Nanodegree Program
Udacity SDCND Term 2, Project 4
---

## Project Basics

This Project is implemented in C++ , I used a Proportional-Integral-Derivative Controller know as PID controller to drive the car around the track in the  simulator. The primary focus of this project was to implent the controller to control the steering angle . However I have also used the controller to control the throttle.

## Reflections

### PID Components

The actual implementation of code for basic PID controller is straight forward i.e 

 -Initializing the constants

-Updating the Error 

-Returning the Total Error

But making the controller to work as desired was the tough part . As for that we must need to have the knowlege of each part . 

P (Propotional) : Means the car will steer with propotion to the cross-track error, or CTE where CTE is the error that determines how far the car is from the  middle of the road. It simply means that when the car is to the right from the moddle of the road then you want to steer to the left and vice versa to keep the car in the middle, for high CTE value we need to have a higher steering angle . But the problem is for a very high Propotional constant value the car will oscillate too much resulting in overshooting and overcorrecting the middle. Following is the formula to calculate the propotional error where Kp is the propotional coefficient

                                                steer = -Kp * cte

I (Integral) : It sum up all the CTE so that the car may  turn back to the middle of the road and avoid it from driving it to one side of the middle the whole time . Too many negative values of CTE will determine that the car have been to the left of the middle for a while and vice-versa. If the coefficint of I is too high then the car may tend to oscillate too much to be in the middle of the road and may not tend to get up a quick speed . A low coefficent for I will cause the car to tend to drift to one side of the lane or the other for longer periods of time. Following is the formula to calculate the integral error where Ki is the integral coefficient

                                              steer = - Ki * int_cte

D (Derivative) : It calculates the change in CTE from one value to the next. This mainly help the car to be on track on the curves as on the rate of change of deravative is high which means higher steering angle . Following is the formula to calculate the derivative error where Kd is the derivative coefficient 

                                             steer = - Kd * diff_cte 
                                             

The total error-value will be given by :

                                    steer = -Kp * cte - Kd * diff_cte - Ki * int_cte

The following gif gives a clear idea of how the change in coefficients of the error effect the final output 

![alt text](./images/coeff.gif)


### Finding the right coefficients

The coefficients were choosen randomly after experimenting on multiple combinations of the PID coeffcients. i.e. (0.2, 0.000, 0.40) This combinations seemed to work well in keeping the vehicle on the track.

For the throttle I choosed a different PID controller so that the throttle value directly depend upon the steer-value i.e the total error . High steer-value means low throttle while low steer-value results in high throttle value. It is given as

                                msgJson["throttle"] = (1 - std::abs(steer_value)) * 0.05 
                                
where 0.05 is choosen randomly to keep the throttle value low.
                           





## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)




