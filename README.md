## Table of Contents
- [Building](#building)
- [Requirements](#requirements)
- [Dependencies](#dependencies)
  - [Eigen](#eigen)
  - [Download MAVLink Libraries](#mavlink-libraries)
  - [DigiMesh](#digimesh-library)
  - [Qt Libraries](#qt-libraries)
  - [Building on ODROID](#building-on-odroid)
- [Command line Compile MACE with UNIX](#mace-compile-unix)
  - [CMake](#cmake)
  - [QMake](#qmake)
- [Develop with Qt IDE on ODROID](#qt-on-odroid)
  - [Install Qt Creator](#install-creator-odroid)
  - [Configure Qt Creator](#configure-creator-odroid)
- [Develop with Qt IDE on Windows](#qt-on-windows)\
  - [Configure Qt Creator](#configure-creator-windows)
- [Running on ODROID](#running-on-odroid)
- [(Optional) ROS Setup](#ros-setup)



# <a name="building"></a> Building

Use information bellow to buld entire source tree for your platform

## <a name="requirements"></a> Requirments
List which MACE has been successfully targeted.
* GCC 4.6.4+ OR MSVC2013
* QT 5.7.0+


## <a name="dependencies"></a> Dependencies

### <a name="eigen"></a> Eigen

Download and install the lastest Eigen library from  
`http://eigen.tuxfamily.org/index.php?title=Main_Page`

For both Unix/Windows the default install location is fine.
For windows you may need to open a `cmd.exe` window with adminstator privaliges, then run `INSTALL.vcxproj` in the build directory such that the "INSTALL" project can copy it's files onto the filesystem.

### <a name="mavlink-libraries"></a> Download Mavlink libraries

The Mavlink library is configured as a submodule in git.
To download the nessessary files, run the following command in a shell on the project.
```
git submodule update --init --recursive
```

### <a name="qt-libraries"></a> Qt Libraries

Currently we utialize QtSerialPort library to faciliate communication to vehicles therefore Qt's libraries are required. However it is not required to use Qt's toolchain to build/develop MACE.

For Windows or "typical" linux install QT as per normal

### <a name="digimesh-library"></a> Digimesh Library

Download and compile Digimesh library distributed by Heron Systems: https://github.com/heronsystems/MACEDigiWrapper

Generated headers and lib should install to a `/headers` and `/lib` directory off the root of the project, this is done by default by doing `make install` on above project.
Add an environment variable `MACE_DIGIMESH_WRAPPER` set to that root such that `%MACE_DIGIMESH_WRAPPER%/include` and `%MACE_DIGIMESH_WRAPPER%/lib` resolves to the appropriate directories. There is no need to add either to the `PATH` environment variable.

#### <a name="building-on-odroid"></a> Building On ODROID

For ODROID, QT will need to be compiled from source due to archetecture.

Throughout these instructions %VERSION% will refer to the version of Qt installing, for me it was 5.7.0, but it may be different depending on your preference.

Install Qt onto the host machine by downloading Qt source from  
`https://www.qt.io/download-open-source/`

The following instructions are largely based on Qt's own instructions: [http://doc.qt.io/qt-5/linux-building.html](http://doc.qt.io/qt-5/linux-building.html).

Prior to building ensure libxcb is installed by running:
```
apt-get install libxcb1-dev
apt-get install libx11-dev
```

To build Qt Tool chain navigate a shell to the unziped folder and run
```
./configure -opensource -confirm-license -qt-xcb
make
make install
```
Next add 
```
PATH=/usr/local/Qt-%VERSION%/bin:$PATH
export PATH
```
to `.profile`, after this change log-off/log-on would be required.

## <a name="mace-compile-unix"></a> Command Line Compile MACE with UNIX

### <a name="cmake"></a> CMake

From the project's root directoy create a build folder
```
mkdir build
cd build
``` 

Next run CMake to generate build scripts
```
cmake -DBUILD_SHARED_LIBS=YES ../src -DCMAKE_PREFIX_PATH=/usr/local/Qt-%VERSION%/
```
To instead place build outputs into projects directory
```
cmake -DBUILD_SHARED_LIBS=YES -DCMAKE_INSTALL_PREFIX="${PWD}/../" ../src -DCMAKE_PREFIX_PATH=/usr/local/Qt-%VERSION%/
```

Finally make
```
make
make install
```

### <a name="qmake"></a> QMake

From the project's root directoy create a build folder
```
mkdir build
cd build
```

Next run CMake to generate build scripts
```
qmake ../src/src.pro
```

Finally make
```
make
make install
```

# <a name="qt-on-odroid"></a> Develop with QT IDE on ODROID

The following steps detail how to develop with Qt on the odroid and how to install and operate qtcreator on the odroid to build MACE.

## <a name="install-creator-odroid"></a> Install QtCreator

To add qt creator use apt-get
```
apt-get install qtcreator
```

Something is apparently wrong with Welcome screen. Start QtCreator with no Welcome screen
```
qtcreator -noload Welcome
```
This change can be perminatly set by navigting to Help->Plugins and unchecking "Welcome"

## <a name="configure-creator-odroid"></a> Configure QtCreator

With QtCreator open the build kit must first be configured.  
Open Options menu under Tools->options. Select "Build and Run" on left side.

* Add Compiler  
Add a new compiler and set "Compiler Path" to be `/usr/bin/arm-linux-gnueabihf-gcc`  
Apply Changes
* Check Qt Version
Under "Qt Versions" tab the entry for Qt %VERSION% in PATH should no longer have a red exlamation point beside.  
If it still does, panic.
* Add Kit
Under "Kits", add new kit.  
Name something relvant, set compiler to the compiler set previously.  
Set Qt version to "Qt %VERSION% in PATH".  
Apply changes.

Now open MACE/src/src.pro ensuring that it is using the kit you just created and build MACE project as you wish.
Under "Project" option on left you may want to check "Shadow build" this helps keep your source directory clean of build artifacts.

# <a name="qt-on-windows"></a> Develop with QT IDE on Windows

The following steps detail how to develop with Qt on windows.

##Install QtCreator

Likely this was installed in the above step to 

## <a name="configure-creator-windows"></a> Configure QtCreator

With QtCreator open the build kit must first be configured.  
Open Options menu under Tools->options. Select "Build and Run" on left side.

* Add Compiler  
Add a new compiler and set "Compiler Path" to be `/usr/bin/arm-linux-gnueabihf-gcc`  
Apply Changes
* Check Qt Version
Under "Qt Versions" tab the entry for Qt %VERSION% in PATH should no longer have a red exlamation point beside.  
If it still does, panic.
* Add Kit
Under "Kits", add new kit.  
Name something relvant, set compiler to the compiler set previously.  
Set Qt version to "Qt %VERSION% in PATH".  
Apply changes.

Now open MACE/src/src.pro ensuring that it is using the kit you just created and build MACE project as you wish.
Under "Project" option on left you may want to check "Shadow build" this helps keep your source directory clean of build artifacts.


# <a name="running-on-odroid"></a> Running on ODROID:
_NOTE_: If you cannot run via command line on an ODROID, try running the following command:

`export LD_LIBRARY_PATH=<path_to_MACE>/lib`

where `<path_to_MACE>` is replaced with the local path to your MACE root directory. This tells the console when running an application to also look in that path for any applicable libraries. 


# <a name="ros-setup"></a> (Optional) ROS Setup
ROS can be used to simulate worlds and sensors. Steps to set up ROS and configure various vehicles can be found in the [ROS Setup](https://github.com/heronsystems/MACE/wiki/ROS---MACE-Setup) wiki page. 
