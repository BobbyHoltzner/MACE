# Electron GUI README

This is the GUI for the ground side of the MACE software suite. The goal of this GUI is to link a user interface to the MACE modules/libraries for display and high-level control of multiple vehicles. 

## Install
To build the GUI from source, you will need [npm](http://npmjs.com) and [gulp](https://github.com/gulpjs/gulp/blob/master/docs/getting-started.md) installed.

Open a new console and change directories into the `MACE/ElectronGUI` directory. Install Node dependencies and build the React project:

```bash
npm i
gulp defaultBuild
```
Note that linking only needs to be done once. If you delete the node_modules folder for whatever reason, re-run the above commands.

Now start the GUI:

```bash
npm start
```
If any changes are made to the React project, you can simply run `gulp defaultBuild` from the `MACE/ElectronGUI` directory and then press `Ctrl + R` in the electron GUI to refresh the application. 

## Prebuilt Binaries
If you do not want to build from source, there are prebuilt binaries included in the `MACE/ElectronGUI/PrebuiltBinaries` directory for several architectures. Simply find your architecture and run the executable in the corresponding directory to start the MACE GUI. The following architectures are supported:

- Win32-x64
- Win32-ia32
- Linux-x64
- Linux-ia32
- Linux-armv7l

## Usage
This GUI is a work in progress, and as such there are certain "quirks" with its usage. The following process has been found to work consistently on a Windows 10 machine:

1. Start the ArduPilot simulation(s) (see this page for reference: [TODO-SIMULATION README](https://www.google.com))
2. Start MACE
3. Start the MACE GUI

The GUI has inconsistent failures when it is started before MACE has knowledge of any vehicles, so it is best to start MACE and the simulation(s) (or actual vehicles) before the GUI is started. 

NOTE: To help with connection issues with MACE, there is a 7-second timer at GUI launch that waits 7 seconds before asking MACE what vehicles it has connected. 

### Selecting a vehicle
Many actions in the GUI require a vehicle ID to be set. To do so, with a vehilce connected, click on the Vehicle HUD on the right-hand side of the screen. This will open a "Set Home Location" dialog where you can set that particular vehicle's home location. If you do not wish to do so, simply close the dialog (by clicking off of it or hitting cancel) and nothing will be sent to the vehicle. At this point however, that vehicle is now your "Selected vehicle," so you can Arm it and command it to change modes. 

Again, this GUI is a work in progress, and one of the main features to be added is an intuitive way to select a specific vehicle or all vehicles. 


## TODO:
- Allow for easier selecting of vehicles
- Display the selected vehicle
- Allow for a user-defined "Sync" with MACE to get all currently connected vehicles
- Takeoff altitude dialog? 
  * This will have to be an "Arm and Takeoff" dialog because the motors will disarm before the user has a chance to type in the value and initiate a takeoff
