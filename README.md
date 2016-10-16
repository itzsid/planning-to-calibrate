Planning-to-Calibrate
===================================================
This library is an implementation of the algorithm described in Active Planning Based Extrinsic Calibration of Exteroceptive Sensors in Unkown Environments (IROS 2016). The core library is developed in C++
and MATLAB.

Planning-to-Calibrate is developed by [Varun Murali](mailto:varunmurali1@gmail.com), [Carlos Nieto](mailto:carlos.nieto@gmail.com) and [Siddharth Choudhary](mailto:siddharth.choudhary@gatech.edu) as part of their work at Georgia Tech. 

Prerequisites
------

- CMake (Ubuntu: `sudo apt-get install cmake`), compilation configuration tool.
- [Boost](http://www.boost.org/)  (Ubuntu: `sudo apt-get install libboost-all-dev`), portable C++ source libraries.
- [GTSAM](https://bitbucket.org/gtborg/gtsam) >= 3.0, a C++ library that implement smoothing and mapping (SAM) framework in robotics and vision.
Here we use factor graph implementations and inference/optimization tools provided by GTSAM. Install the MATLAB wrapper as well. 

Compilation & Installation
------

In the planning-to-calibrate folder excute:

```
$ mkdir build
$ cd build
$ cmake ..
$ make -j3
$ make check  # optonal, run unit tests
$ sudo make install # this will install the matlab wrapper
```

Run Experiments
------

```
$ cd matlab/PlanningToCalibrate
$ run SLAMGUI.m 
$ press on Run Experiments
```

Questions & Bug reporting
-----

Please use Github issue tracker to report bugs. For other questions please contact  [Varun Murali](mailto:varunmurali1@gmail.com), [Carlos Nieto](mailto:carlos.nieto@gmail.com) and [Siddharth Choudhary](mailto:siddharth.choudhary@gatech.edu).

Acknowledgements
----
This work was partially funded by the ARL MAST CTA
Project 1436607 “Autonomous Multifunctional Mobile Microsystems”

Citing
-----

If you use this work, please cite following publication:

```
@inproceedings{Murali16iros,
  author    = {Varun Murali and
               Carlos Neito and
	       Siddharth Choudhary and
               Henrik I. Christensen},
  title     = {Active Planning based Extrinsic Calibration of Exteroceptive Sensors in Unknown Environments},
  booktitle = {2016 {IEEE/RSJ} International Conference on Intelligent Robots and Systems, Hamburg, Germany},
  year      = {2016}
}
```


License
-----

Planning-to-Calibrate is released under the BSD license, reproduced in the file LICENSE in this directory.
