# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)

[image1]: ./examples/PID_block.png "PID_block"

## Project Goal

In this project, the goal is to program a PID controller in C++. The PID controller takes CTE (cross track error) from the simulator. The simulator could be downloaded here: https://github.com/udacity/self-driving-car-sim/releases. The output of PID controller is to control the steering angle and/or throttle for a car running around a circular track. The hyperparameters (PID) will be optimized by twiddle method taught in the course.

## Background

A proportional-integral-derivative controller (PID controller) is a control loop feedback mechanism widely used in industrial control systems. The distinguishing feature of the PID controller is to use three control terms of proportional, integral and derivative influence on the controller output to apply accurate and optimal control.

Below shows a PID control block diagram.

![alt text][image1]

## Rubric Disussion points

*Student describes the effect of the P, I, D component of the PID algorithm in their implementation.*

* Term P is the proportional term. It is proportional to the current value of CTE. If the CTE is large and positive, the control output will be proportionately large. If the car is far to the left it steers hard to the right, if it's slightly to the right it steers slightly to the left.

* Term I is the integral term. It accounts for past values of CTE and integrates them over time. The I term counteracts a bias (for example, a steering drift) in the CTE.

* Term D is the derivative term. It is an estimate of the future trend of the CTE, which is based on its current rate of change. The more rapid the CTE change, the greater the controlling or dampening effect. In this project, It will help the vehicle to approach the center line smoothly without ringing.       


*Student discusses how they chose the final hyperparameters (P, I, D coefficients).*

The hyperparameters were tuned manually first before applying Twiddle optimization. This is necessary to make sure less chance for the vehicle to leave the circular track. Then I applied the twiddle algorithm learned from Sabastian. Twiddle is an algorithm that tries to find a good choice of hyperparameters for a PID algorithm that returns an error. The twiddle algorithm is to evaluate each change of PID hyperparameters in a full lap (I used 1800 steps for the evaluation. 1800 steps are very close to 1 circular lap).

For the throttle, instead of using a constant throttle value, I make it as a function of steering angle: `throttle_value = 0.6 - std::abs(steer_value)`. If there is a larger steering angle target, the throttle will be reduced to slow down the vehicle. If there is a smaller steering angle commanded, the throttle will be increased up to 0.6. The vehicle will speed up and maintain at 0.6 throttle.    


### Final Parameters

You can find a [video](./examples/PID_Control.mp4) with the final PID values. It's located in the *./examples/PID_Control.mp4*.
The final PID values for the above demo video is **(P: 0.11, I: 0.00009, D: 3.0)**.


# Below is Original Udacity README
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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
