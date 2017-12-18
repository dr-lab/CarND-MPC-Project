# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---


## How Project Works

This project will start a service on port 4567, which listens any incoming data from the same simulator used in previous projects. Data is communicated by uWebSockets.

* **Sample incoming data from Simulator**

    42["telemetry",{"ptsx":[117.2083,98.34827,83.63827,79.68355,78.52827,77.04827],"ptsy":[-69.827,-42.02898,-20.72898,-12.66062,-7.878983,-1.338982],"psi_unity":5.720081,"psi":2.1339,"x":110.1315,"y":-59.58069,"steering_angle":0.00938553,"throttle":0.4471804,"speed":51.50375}]

When the service got the data from port 4567, it will parse it, get values of [ptsx, ptsy,psi_unity, psi, x, y, steering_angel, throttle and speed].

After get the data, the service will do some data transformation, calculate the reference path and the MPC trajectory path which will be sent back to simulator, and been visualized on the simulator UI.

    42["steer",{"mpc_x":[10.3043770180367,15.4600377650319,20.6106998671241,25.7600298427125,30.9133558683391,36.0732754356507,41.2390126837818,46.4078560527436,51.5768923169674],"mpc_y":[-0.093320795018423,0.100578280193056,0.461280788249331,0.885218672954205,1.29673096812346,1.65311892528547,1.94889733528375,2.21497357494321,2.49619537580771],"next_x":[0.0,2.5,5.0,7.5,10.0,12.5,15.0,17.5,20.0,22.5,25.0,27.5,30.0,32.5,35.0,37.5,40.0,42.5,45.0,47.5,50.0,52.5,55.0,57.5,60.0],"next_y":[-1.19531198307524,-1.12012433464043,-0.992012114314628,-0.818771468107446,-0.608198542028485,-0.36808948208735,-0.106240434293645,0.169552455343026,0.451493040813059,0.73178517610685,1.00263271521479,1.25623951212729,1.48480942083472,1.6805462953275,1.83565398959602,1.94233635763066,1.99279725342184,1.97924053095993,1.89387004423535,1.72888964723848,1.47650319395973,1.12891453838948,0.678327534518129,0.116946036336078,-0.56302610216628],"steering_angle":-0.0661150200976763,"throttle":0.445882655482734}]


* **Process and Calculate Cost**

  In the **main.cpp** L156, which calls one method in MPC.cpp

      auto vars = mpc.Solve (state, coeffs);

  This Solve() method is the key of MPC project, which first calculate the cost (fg[0]) . For more details of the explainatin, please check Lession 19-9.


        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)

        // cost is stored in first element of fg
        fg[0] = 0;


        // The part of the cost based on reference state
        for (int t = 0; t < N; ++t) {
            fg[0] += 200 * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
            fg[0] += 200 * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
            fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
        }

        // Minimize the use of actuators
        for (int t = 0; t < N - 1; ++t) {
            fg[0] += 80000 * CppAD::pow(vars[delta_start + t], 2);
            fg[0] += 500 * CppAD::pow(vars[a_start + t], 2);
        }

        //Minimize the value gap between sequential actuations

        for (int t = 0; t < N - 2; ++t) {
            fg[0] += 200 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }

    Note some parameters used on the CppAD::pow(). Multiplying that CppAD::pow() by a value > 1 will influence the solver into keeping sequential steering values closer together. Empirically the value we used above can finish the loop successfully on 50m/h speed.

    In general, we want the steering angle values to be smooth. If the vehicle is behaving erratically it's likely there will be spikes in the steering angle value graph and its one indication you should tune the cost function.


* **Rubic Requirements**

**The Model:**

The kinematic model discussed in the class room includes the vehicle's x and y coordinates, orientation angle (psi), and velocity, as well as the cross-track error and psi error (epsi). Actuator outputs are acceleration and delta (steering angle). The model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on the equations below:

     Recall the equations for the model:
         x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
         y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
         psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
         v_[t+1] = v[t] + a[t] * dt
         cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
         epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt


Note: Equations are from the lessons.


**Timestep Length and Elapsed Duration (N & dt):**

The values chosen for N and dt are 10 and 0.1, respectively.  This is defined in MPC.cpp file line 9-10.

        size_t N = 10;
        double dt = 0.1;

This was the suggestion of Udacity's provided office hours for the project, and with the test run in local simulator, these values can help me finish the circle with good speed . These values mean that the optimizer is considering a one-second duration in which to determine a corrective trajectory. Adjusting either N or dt (even by small amounts) often produced erratic behavior.

** Polynomial Fitting and MPC Preprocessing: **

The waypoints are preprocessed by transforming them to the vehicle's perspective.

        // start by transforming the points to be in car's co-ordinate system
        // shift car reference angle to 90 degrees
        for (int i=0;i < ptsx .size(); i++)
        {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            ptsx[i] = (shift_x * cos (0-psi) - shift_y * sin (0 - psi));
            ptsy[i] = (shift_x * sin (0-psi) + shift_y * cos (0 - psi));
        }

This simplifies the process to fit a polynomial to the waypoints because the vehicle's x and y coordinates are now at the origin (0, 0) and the orientation angle is also zero.


Next we have to find the coefficients for fitting the polynomial.


      //We have to makesure we pass vectors as Eigen vectors topolyfit
      double* ptrx = &ptsx[0];
      Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

      double* ptry = &ptsy[0];
      Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

      //then we do the polyfit. polyfit defined in main.cpp line 47-66
      auto coeffs = polyfit (ptsx_transform, ptsy_transform, 3 );


** Model Predictive Control with Latency: **

A contributing factor to latency is actuator dynamics. For example the time elapsed between when you command a steering angle to when that angle is actually achieved. This could easily be modeled by a simple dynamic system and incorporated into the vehicle model. One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.Thus, MPC can deal with latency much more effectively, by explicitly taking it into account, than a PID controller.

The approach to dealing with latency was twofold: the original kinematic equations depend upon the actuations from the previous timestep, but with a delay of xxms (which happens to be the timestep interval) the actuations are applied another timestep later. Thus the optimal trajectory is computed starting from the time after the latency period. This has the advantage that the dynamics during the latency period is still calculated according to the vehicle model.


## Run and Test

The yellow is a polynomial fitted to waypoints and the green line represents the x and y coordinates of the MPC trajectory. In the local test run, we can see the speed is little above 50m/h.

[![PID driving Video](https://img.youtube.com/vi/dKI5EoLLmEc/0.jpg)](https://youtu.be/dKI5EoLLmEc)

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

* **Ipopt and CppAD:**

This project needs to install Ipopt and CppAD. I use brew install both of them

    brew install ipopt
    brew install cppad

MacOS I am using is High Sierra 10.13.1 (17B1003).


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Call for IDE Profiles Pull Requests

I use XCode to edit and debug. In the root of the project folder, run following command:

    cmake -G "Xcode" .

## Others


In this project, the algorithm use [Ipopt::CppAD](https://www.coin-or.org/CppAD/Doc/doxydoc/html/namespaceCppAD_1_1ipopt_a2b3f613ffa4b230d7c407d3c04c64dd4.html) to Solve a Nonlinear Programming Problem. And in the MPC.cpp use C++ operator overloading.
