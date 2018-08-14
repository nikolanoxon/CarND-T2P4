# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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


## Project Rubric 

### Describe the effect each of the P, I, D components had in your implementation.

For this project I implemented both a lateral and longitudinal controller to control the steering angle and throttle respectively.

#### Lateral Control

* P - Minimized the cross-track error. However, too large of a gain would cause large oscillations of the vehicle around the setpoint.

* I - Minimized the error-over-time of the CTE. This gain allows the vehicle to achieve a CTE of zero which P control alone cannot do. If incorrectly parameterized, could cause very large over-corrections when the vehicle is offset from the CTE for an extended period of time (like during a turning maneuver). 

* D - Minimized the rate-change of the CTE. Basically a "first responder" to a large steering requirement, this gain ensured that the vehicle turned in time during a curve. If too large, this parameter would cause the vehicle to over-steer. This parameter is very sensitive to the types of turns experienced on the track. It's likely that if tighter turns existed that this parameter would need further tuning.

#### Longitudinal Control

* P - Minimize the difference between the vehicle speed and the set speed. If too large of a parameter was used this would cause large oscillations in the vehicle speed.

* I - Minimize the error-over-time of the set-speed delta. This helps ensure that the set-speed is eventually reached over time.

* D - Minimize the rate-change of speed to keep the acceleration smooth and reduce jerk.

### Describe how the final hyperparameters were chosen.

The final parameters used for this project were:

#### Lateral
* P = 0.190967
* I = 0.000159967
* D = 2.1

#### Longitudinal
* P = 0.1
* I = 0.00001
* D = 0.1

#### Tuning

I began by tuning the parameters by hand, then once I'd found parameters that seemed to be close, I used Twiddle to fine tune the results. For the lateral control:
* I started with only proportional control and gradually increased the gain until the vehicle oscillated.
* Cut the proportional control in half
* Added a very small integral gain to allow the vehicle to reach the setpoint.
* Added derivative gain until desirable behavior was observed.

I followed a similar pattern for the longitudinal control:
* Proportional control until the speed oscillated
* Cut the P gain in half
* Added a small I gain
* Added D gain until the speed controller was steady

After manual tuning, the values I obtained were:

#### Lateral
* P = 0.-75
* I = 0.0001
* D = 3.0

#### Longitudinal
* P = 0.1
* I = 0.00001
* D = 0.1

At this point I decided to implement the twiddle algorithm on both controllers. However I found that tuning the longitudinal controller was not really improving the performance significantly and was doubling the time it took to tune so I disabled it. Using the manually tuned gains as a starting point, I ran the simulation over the whole course and obtained the following after 460 iterations:

* STEERING Kp: 0.190967 Ki: 0.000159967 Kd: 2.1
* STEERING Iteration: 460 Active Gain: 0 Gain State: 0
* STEERING cycle: 1901 Current Error: 612.552

### Conclusion
In the end I was able to run the vehicle at a throttle target of 0.6, which comes out to around 50 mph. I included a steering angle saturation of +/- 20 degrees since that tended to oversteer the vehicle. If I were to add further improvements, I would add a controller which adjusted the throttle by using CTE as the error signal. I think this would help the vehicle slow down in curves. Additionally I would reexamine my proportional tuning since the vehicle wanders quite a bit on the track.