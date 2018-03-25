# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Report


### Coefficients, P, I, D.

* P controls the response for immediate error.
  Large P indicates that control follows very responsively from **cte**.
  But it sometimes also means dramastical change and oscillation since car's behaviors has delay after control
* I is sum of historical error, it is the biase of the control to the reference.
  This value can help long term running and average small error to the reference trajectory.
  Usually this value is very small.
* D controls the difference between two consecutive error, it smoothes the control and car's behaviors


### Tunning Parameters

All the control is tuned automatically by **twiddle algorithm PID::Twiddle**.
Final parameters for two pids (pid, speed pid) are following:

```python
  PID pid;
  // TODO: Initialize the pid variable.
  std::vector<double> coeffs{0.855083, 0.000418873, 0.153373};
  std::vector<double> d_coeffs{2.75347e-05, 9.99717e-06, 2.50316e-05};
  pid.Init(coeffs);

  // the second is the maximum cte allowed, decrease it to improve the quality
  // set first parameter to false to turn on auto tunning
  pid.InitTwiddle(true, 0.00001, 1.0, 100, 2000, d_coeffs);

  PID speed_pid;
  std::vector<double> coeffs2{0.208562 , 0.00033317 , 0.196767};
  std::vector<double> d_coeffs2{0.00147042, 0.000115008, 0.00100433};
  speed_pid.Init(coeffs2);
  // for speed, we set the sixth 0
  speed_pid.InitTwiddle(true, 0.0001, 20, 100, 2000, d_coeffs2);
```

There are two PIDs, I first set speed_pid parameters to (0.1, 0, 0) since it is more easily set manually
and tune the core pid for controlling. After finding a good enough parameters for pid, then tune speed pid,
though it seems not too much space to improve.

I also build the target speed according to steering value like following.

```python
  double target_speed = 20.0 * (1.-abs(steer_value)) + 10.0;
  double speed_cte = (speed - target_speed);
```


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


## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

