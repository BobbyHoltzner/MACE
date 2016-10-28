# Electron GUI README

This is the GUI for the ground side of the MACE software suite. The goal of this GUI is to link a user interface to the MACE modules/libraries for display and high-level control of multiple vehicles. 

TODO:

- Link with the ArduPilot simulation environment to remove the need for hardware.

## Install
To build the GUI from source, you will need [npm](http://npmjs.com) and [gulp](https://github.com/gulpjs/gulp/blob/master/docs/getting-started.md) installed. You will also need [SWIG](http://www.swig.org/Doc1.3/Windows.html) if you plan to make any changes to the mace-api c++ Node package.

Open a new console and change directories into the `MACE/node_packages/mace-api` directory. Link the package:

```bash
npm link
```
This will tell npm where the mace-api package is to later be linked to the ElectronGUI.

Open a new console and change directories into the `MACE/ElectronGUI` directory. Install Node dependencies, link the `mace-api` package, and build the React project:

```bash
npm i
npm link mace-api
gulp defaultBuild
npm run rebuild
```
Note that linking only needs to be done once. If you delete the node_modules folder for whatever reason, re-run the above commands.

Now start the GUI:

```bash
npm start
```
If any changes are made to the React project, you can simply run `gulp defaultBuild` from the `MACE/ElectronGUI` directory and then press `Ctrl + R` in the electron GUI to refresh the application. 


### mace-api Node package
The linking steps above will allow you to make changes in the `node_packages/mace-api` directory and have them be reflected in the ElectronGUI's node_modules directory. Howeveer, if any changes are made, you will need to rebuild the `mace-api` node package for electron. Open a new console and change directories into the `MACE/node_packages/mace-api` directory. Run SWIG to re-build the C wrapper for Node:

```bash
swig -c++ -javascript -node ./src/mace_api.i
```
Now change directories to `MACE/ElectronGUI` and rebuild the package for electron:

```bash
cd ../../ElectronGUI
npm run rebuild
```
