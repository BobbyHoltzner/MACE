# Voronoi-Based Coverage Control for Multiple Aerial Vehicles
This is a part of framework simulation that generate cooperative path plan,and allocate it to each UAV. Its take each UAV states,and active nodes states as input. Each vehicle's distance from the every node is computed. Each point is assigned to the closest vehicle according to a Voronoi partition as output.

This c++ code will use openFrameworks and Eigen open source libaries.

The easiest way to get use this code/libraries is to install the Visual Studio.

[openFrameworks](http://openframeworks.cc/)
================

openFrameworks is a C++ toolkit for creative coding.  If you are new to OF, Follow the quick steps.

Setup
--------

Visual studio Setup guide [click here](http://openframeworks.cc/setup/vs/)

Visual studio [![Build status](https://ci.appveyor.com/api/projects/status/sm9jxy0u56bl8syi/branch/master?svg=true)](https://ci.appveyor.com/project/arturoc/openframeworks/branch/master)

[Eigen](http://eigen.tuxfamily.org/)
================

Eigen is a C++ Template libray for linear algebra:matrices, vectors, numerical solvers, and realted algorithms.

Setup
--------

[Download](http://eigen.tuxfamily.org/index.php?title=Main_Page) the eigen and folllow the steps.

Visual studio Setup guide [click here](http://eigen.tuxfamily.org/index.php?title=Visual_Studio)

Output:
================

Now you are ready to build the code in visual studio.

Go ahead build the project.

Note for GUI:
* "Q-key" for Play/Pause the simulation.
* "X-key" for screenshot
* "P-key" for planning thread if code is in manual macro.

Please email bug reports, comments or code contribtions to me at ravib at terpmail dot umd dot edu or contact me on +1-772-222-RAVI
