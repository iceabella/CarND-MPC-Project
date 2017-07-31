# CarND-Controls-MPC
This is my implementation of the MPC project for Udacity's Self-Driving Car Engineer Nanodegree Program, the last project of term 2.

## The model
The vehicle is in this project modelled by a kinematic bicycle model. The state of the vehicle is described by:

```
x(k) - x position of vehicle in time step k relative to vehicle coordinate system
y(k) - y position of vehicle in time step k relative to vehicle coordinate system
psi(k) - heading of vehicle in time step k relative to vehicle coordinate system
v(k) - velocity of vehicle in time step k
cte(k) - estimated cross track error in time step k, here calculated in vehicle coordinates as the difference between y(k) of the vehicle and y position of reference trajectory evaluated in point x(k) 
epsi(k) - estimated heading error in time step k, here calculated as the difference between psi(k) of the vehicle and the desired heading according to the reference trajectory heading in x(k)
```

The control inputs are:
```
delta(k) - steering input in time step k
a(k) - throttle/brake pedal input in time step k
```
Note that throttle/brake pedal input is not the same as acceleration, but in this project it is estimated to be the same.

The Update equations describes how the vehicle state is estimated to change in future time steps, the equations used for the project are based on a simplified bicycle model:
```
x(k+1) = x(k) + v(k) * cos(psi(k)) * dt
y(k+1) = y(k) + v(k) * sin(psi(k)) * dt
psi(k+1) = psi(k) - v(k) / Lf * delta(k) * dt
v(k+1) = v(k) + a(k) * dt
cte(k+1) = cte(k) + v(k) * sin(epsi(k)) * dt = f(x(k)) - y(k) * sin(epsi(k)) * dt
epsi(k+1) = epsi(k) - v(k) / Lf * delta(k) * dt = psi(k) - atan(f'(x(k))) - v(k) / Lf * delta(k) * dt
```

where `dt` is the time interval between predictions, `f(x(k))` is the desired y value on the reference trajectory (y value for x(k)), `atan(f'(x(k)))` is desired heading based on the heading/derivative of the reference trajectory in x(k) 

NOTE1: Since in the simulator a positive steering value implies a right turn and a negative value implies a left turn I have used steering actuation (delta) * -1 wherever present in the equations.
NOTE2: For psi(k+1) (as well as espi(k+1) a small angle approximation is made (since we are restricting the angle between [-25deg, 25 deg]). Additionally the slip angle is assumed to be zero, which is reasonable for the simulator BUT in real applications this is only valid for low speeds (less than 5m/s).
NOTE3: the calculation of cte(k) = f(x(k)) - y(k) is an approximation, in reality this is only true if the heading of the vehicle is the same as the reference trajectory heading.


## MPC problem
The MPC problem we would like to solve is to minimize our cost function (seen below) subject to the constraints that our next state is equal to the estimated next state. The minimization problem in mathematical terms:

```
min: Sum (i=0,1,...,N) [ a1*cte(k+i)^2 + a2*epsi(k+i)^2 + a3*(v(k+i)-v_ref)^2] + Sum (i=0,1,...,N-1) [a4*delta(k+i)^2 + a5*a(k+i)^2 + a8*(cte(k+1+i)-cte(k+i))^2 + a9*(epsi(k+1+i)-epsi(k+i))^2] + Sum (i=0,1,...,N-2) [a6*(delta(k+1+i)-delta(k+i))^2 + a7*(a(k+1+i)-a(k+i))^2]

subject to:
x(k) = x(k)
y(k) = y(k)
psi(k) = psi(k)
v(k) = v(k) 
cte(k) = cte(k)
epsi(k) = epsi(k)
for i=0 to N-1:
	0 = x(k+1+i) - (x(k+i) + v(k+i) * cos(psi(k+i)) * dt)
	0 = y(k+1+i) - (y(k+i) + v(k+i) * sin(psi(k+i)) * dt)
	0 = psi(k+1+i) - (psi(k+i) - v(k+i) / Lf * delta(k+i) * dt)
	0 = v(k+1+i) - (v(k+i) + a(k+i) * dt)
	0 = cte(k+1+i) - (cte(k+i) + v(k+i) * sin(epsi(k+i)) * dt)
	0 = epsi(k+1+i) - (epsi(k+i) - v(k+i) / Lf * delta(k+i) * dt)

for i=0 to N:
	-inf <= x(k+i), y(k+i) psi(k+i), v(k+i), cte(k+i), epsi(k+i) <= inf 
	-deg2rad(25)<= delta(k+i) <= deg2rad(25)
	-1.0 <= a(k+i) <= 1.0
```
where `N` is number of prediction steps and `ai` (i=1,2,...,9) are tunable parameters which will influence what is of most importance for our solution. The cost function tuning will be described futher in the following section.

## Parameter tuning
In this project both time parameters and cost function influence needs to be tuned. First out are the time parameters N (number of prediction steps) and dt (time interval between predictions). These parameters are connected since depending on their values we will predict N*dt seconds into the future. If we predict too far into the future the predictions will most likely be incorrect, but if we predict too short we will not have enough data to draw good estimations. Another perspective to take into account is that the higher the N, the bigger our minimization problem will be, i.e. the longer time it will take to compute. 

I think it is most important to have enough (but not too many) prediction steps (N), so I started by setting N=20. I chose dt=0.1, which means a prediction horizon of 2 s. Note that these values are highly influence by velocity since we will travel further during the same time if we are driving in a higher velocity. If we want to drive faster it is usually preferable to sample faster (use a smaller dt). When trying these values in the simulator I realised that the prediction was too far into the future why I shortened it to 10 prediction steps.

During further tuning of cost function parameters and when increasing speed of the vehicle N was changed further. When driving with a top speed of 99mph and above N needed to decrease to 8 to be able to drive around the track, later with top speed 110mph it was decreased to 7.

When latency was added the prediction time interval dt was increased to 0.15 which seemed to work better. I think the reason is that 0.15 is closer to the time it takes between predictions for my PC when latency is added and therefore the control inputs will be more correct. 8 number of predictions was used since only a lower speed was able to be maintained.

The cost function influence is updated by changing the parameters ai (i=1,2,...,9) in the cost function described in the previous section. The higher value of the parameter, the higher influence (intuitively since we want to minimize the cost funtion). 

I focused on getting a smooth steering actuation, why I started to set a high value on the use of steering actuation (I did not think that speed was of high importance). This means that we will aim for lower steering values: 

``` c++
    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += 1000*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }
```

I thereafter increased the importance of a low value gap of the steering actuation between sequential actuations, this to get a smoother transitions between steering values:

``` c++
// Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += 200*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
``` 

To decrease oscillations I added a new part in the cost function, I was thinking that it is of more importance to steer smooth to decrease the error i.e. keep the error similar between different time steps, instead of reaching a small error directly. This can of course keep the error big for a longer time so care must be taken:

``` c++
    // Minimize the value gap between sequential errors
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += 100*CppAD::pow(vars[cte_start + t + 1] - vars[cte_start + t], 2);
      fg[0] += 200*CppAD::pow(vars[epsi_start + t + 1] - vars[epsi_start + t], 2);
    }
```

I didn't add a lot of importance on the cte or epsi errors since I think it started to overshoot too much, instead I was controlling the errors with the sequential error equations instead. I was increasing the cte some to decrease the error to the reference line and the epsi so that we make sure that we are more paralell to the reference line:

``` c++
    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      fg[0] += 10*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 2*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
``` 

As seen all the parameters connected to acceleration and speed where kept low, this to make sure that low importance is given to speed and that sometimes the system can choose to brake the vehicle and reduce the speed to be able to take a turn better for example instead of keeping the speed.


# Polynomial fitting and MPC preprocessing
When receiving the data from the simulator transformations were made of the velocity from mph to m/s and the previous steering value was transformed from [-1,1] to [deg2rad(-25),deg2rad(25)]. Note that during the whole project the assumption was that acceleration was directly proportional to throttle value (this is not correct in reality).

Since the vehicle and map information is received in a global coordinate system a coordinate transform to the vehicle coordinate system was made. This is done by converting the reference trajectory points to vehicle coordinates according to:
``` 
x_rf_v = (x_rf_g - x_g) * cos(-psi_g) - (y_rf_g - y_g) * sin(-psi_g);
y_rf_v = (x_rf_g - x-g) * sin(-psi_g) + (y_rf_g - y_g) * cos(-psi_g);
```
where `(x_rf_v, y_rf_v)` is the position in vehicle coordinates of the global reference point `(x_fr_g,y_rf_g)`, `psi_g` the global heading of the vehicle and `(x_g,y_g)` the global position of the vehicle. This means that the vehicle state has changed for (x, y, psi) to (0, 0, 0).

A third order polynomial is thereafter fit to the reference points in vehicle coordinates.

The state is also updated because of latency before sent into the MPC solver, which will be described further in the next section.

# Latency handling
To resemble a real system a latency of 100ms was added (which on my PC resulted in a total latency of around 150ms). This delay will cause the vehicle to predict from an earlier state than it actually is at when actuating the calculated controls. To correct this I was predicting where the vehicle will be after the latency time by using the kinematic update equations described in "The Model" section, but with the use of latency time as dt. This new state was thereafter seen as the first state and the next predictions were based on this state, i.e. this was the state I was sending into the MPC controller. Note that both this state and all the further estimations was still in the reference frame of the vehicle position when the measurement was received.

To be able to calculate the new state the last steering and throttle values were used from the simulator. 

By introducing this latency I experienced a big loss of information to the controller, the biggest influence can be seen in quickly changing scenarios e.g. curve driving. In these parts bigger overshoots and oscillations did occure. Without latency the vehicle was able to navigate safely upto 110mph, while with latency it can only handled half of that speed.


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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you also need to have installed gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.


