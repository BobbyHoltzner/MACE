## Table of Contents
- [Description](#description)
- [Quick Start](#quick-start)
- [Acknowledgements](#acknowledgements)
- [LICENSE](#license)

# <a name="description"></a> Description
The Multi-Agent Cooperative Engagement (MACE) framework is a framework linking the communications, control, automation and human-machine interface components of a practical multi-vehicle system into a deployable package. MACE establishes the data management, scheduling, and monitoring required by a multi-vehicle robotic system.  Designed as a modular software architecture, MACE implements a core collaborative engine that exposes interfaces with other system components via APIs. This approach abstracts the details of the collaboration away from an individual system, allowing for rapid integration of third party components.

![MACEArchitecture](https://github.com/heronsystems/MACE/blob/master/docs/images/maceArchitecture.png)

The modular architecture of MACE allows the software to be agnostic to the inner workings of a vehicle and its motion primitives. MACE implements a common interface between vehicle communications and the core software, and exposes a flexible API for vehicle developers to interface with other MACE enabled vehicles. Fundamentally, MACE is the backbone communications architecture that can facilitate a Swarm architecture (i.e. vehicle-to-vehicle communications, both 1-to-1 and 1-to-many). MACE provides a simple user interface, however MACE also implements a modular ground station API. Similar to the vehicle communications, this provides methods for third party developers to implement their own human-machine interfaces. Finally, MACE provides a resource and task allocation (RTA) API. This abstracts the tasking of a single vehicle within a swarm or the entire collective swarm to simple waypoints and commands. MACE can be applied to air vehicles, ground vehicles, surface vehicles, or a mixture of different vehicle types. The MACE architecture has been under active development since 2015 and is currently maturing alongside path planning and RTA research programs.

# <a name="qucik-start"></a> Quick Start
A more in depth installation guide is located on the MACE Wiki here: [Install](https://github.com/heronsystems/MACE/wiki/Install). For a quick start, You will need an ArduCopter simulation running, the MACE GUI running, and a simple MACE instance. 

First you have to clone MACE:

```
$ cd <desired>/<path>
$ git clone https://github.com/heronsystems/MACE
```
Once downloaded, you can build MACE using the [Qt Creator IDE](https://www.qt.io/download-qt-installer?hsCtaTracking=9f6a2170-a938-42df-a8e2-a9f0b1d6cdce%7C6cb0de4f-9bb5-4778-ab02-bfb62735f3e5) packaged with Qt. Simply navigate to `MACE/src/` and open `src.pro`. MACE can be compiled using [MinGW](http://mingw.org/)--simply configure your project to build with MinGW in the creator Project menu. Run `QMake` and then `Build` on the project. 

Run MACE from the Qt Crator IDE and it will load the default XML file (located in `MACE/MaceSetup_Configs/Default.xml`). Make sure to change the "ListenAddress" in the configuration file to the IP of your machine.

## <a name="ardupilot"></a> ArduPilot simulation
Follow the instructions [HERE](https://github.com/heronsystems/MACE/wiki/ArduPilot-Simulation) to install ArduPilot SITL. To run a simulated vehicle, run the following in whatever directory you cloned your ardupilot repository in:

```
$ cd ardupilot/ArduCopter
$ sim_vehicle.py
```

## <a name="mace-gui"></a> MACE GUI
To run the MACE GUI, you will need [NodeJS](https://nodejs.org/en/) installed and configured. Once configured, navigate to `MACE/ElectronGUI/` and run `npm install`. Once the installer installs the required Node packages, run the following two commands in separate terminals:

```
**Terminal 1:**
$ npm run watch

*** Terminal 2:**
$ npm run start
```
If successful, the MACE GUI will run, and you should see a vehicle connected similar to below. Note: You may have to adjust the position of the map or the simulated vehicle.

![QuickStartGUI](https://github.com/heronsystems/MACE/blob/master/docs/images/quickStartGUI.png)


# <a name="acknowledgements"></a> Acknowledgments
Heron Systems would like to akcnowledge the following individuals for their contributions on this software:
- Dr. Derek Paley, University of Marlyand, College Park
  - Dr. Paley provided subject matter expertise and algorithm development pertaining to resource and task allocations. Iterations of the algorithms developed with Dr. Paley's guidance were demonstrated both in simulation and onboard COTS platforms
- Robert Waring, VA Department of Conservation and Recreation
  - Mr. Waring was instrumental in providing opportunities to flight test our software while we provided precision agriculture assistance

# <a name="license"></a> LICENSE
**TODO**
