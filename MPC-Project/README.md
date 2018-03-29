# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Report

### The Model

Here, I want to mention two things about the model.

* **Map or Car Coordinate**

The first time, I implemented the MPC using using global map coordinates, then it is not working.
I see some discussion proposal using Car's coordinate for fitting the polynomial and optimization.

Actually I am a little confused about this transformation. For fitting polynomial, defintely
there is no difference. For car's predict model, I think it also works for both coordinates, since
even we transformed to the car's coordinate, everything fed into the optimization is only relative to
the car's starting point `px`, 'py' and 'psi'.
That means the equation of predict model is relative to `(px, py, psi)`,
which we can consider as a global coordinate.
Definitely we do not update the car's coordinate for every predicted steps in the optimizer.

The only advantage of transformation is calculating the right `cte`,
but this only works if the car's starting point is relatively parallel to the reference/the road.
If starting point is vertical to the reference line, for example, then it will fail too.

* **Positive or Negative**

  * Sign of the `psi` and `steering angle`: `psi` is relative to global coordinate, car turning left will
  make it larger. However, `steering angle` for car turning left is negative.

  * Sign of 'cte' and 'epsi': From the class of PID, we know the cte is just `y_c` (car's y) since the reference line
  always has `y_r = 0` (reference's y). Mathematically, that means `cte = y_c - y_r` instead of ``cte = y_r - y_c``.
  In our model equation, we are using both which cause a little confusing for me, though it does not matter
  once it is consistent for all calculation. Code is listed according to our equations for `cte` and `epsi`
  in the lecture

  ```python
  fg[1 + cte_start + t] = cte1 - (polyeval(x0) - y0 + v0 * CppAD::sin(epsi0) * dt);
  fg[1 + epsi_start + t] = epsi1 - (psi0 - CppAD::atan(derivative(x0)) + v0 * steer0 * dt / Lf);
  ```

  If we look carefully, `cte` is calculated as 'desired value minuses current value' but the other way around
  for `epsi`.


### Timestep Length and Elapsed Duration

Timestep Length `N` and Elapsed Duration `dt` controls whether our actions are long term or short term.

If small `dt` and `N` are implemented, it means car might be responsive and does not care long term planning.
The problem for this is that car might be very unstable and swift dramastically.

However, If large `dt` and `N` are implemented, it means car care about long term planning, but might neglect
current situation. The problem is that car might affect dramastically by a very future step.

A good choice of `dt` and 'N' depends on the trail.
A straight and flat trail is good enought to use small 'dt' and `N`,
but winding trail, a relative larger `N` is needed. In my project, following parameters are chosed:

```python

size_t N = 8;
double dt = 0.15;

```


### Model Predictive Control with Latency

Latency will affect our model when the speed is high, we can see the yellow line and gree line is swifting
after that. The reason for that is when speed is high, when the actuators take affect (after 0.1 s), the
car is already for away from the point `(px, py, psi)` we did the coordinate transformation.

Here is how we handle the latency. Basically we assume the starting point, also the initial state point,
is the point after car running the latency time.


```python
double px = j[1]["x"];
double py = j[1]["y"];
double psi = j[1]["psi"];
double v = j[1]["speed"];

// steering left is negative in car coordinates, but for map coordinates, it is positive
// this can be used to derive next state
double steer_angle = j[1]["steering_angle"];
double throttle = j[1]["throttle"];

bool using_latency = false;
if (using_latency == true){
  // predict state in 100ms
  double Lf = 2.67;
  double latency = 0.1;
  px = px + v * cos(psi) * latency;
  py = py + v * sin(psi) * latency;
  psi = psi + v * (0 - steer_angle) / Lf * latency; // change sign of steer_angle
  v = v + throttle * latency; // Note: throttle is not acceleration
}
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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.


## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
