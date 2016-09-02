# MACE
Modular Swarm Optimization Framework Enabling Multi-Vehicle Cooperative Path Planning

##Building

Use information bellow to buld entire source tree for your platform

###Requirments
List which MACE has been successfully targeted.
* GCC 4.6.4 or later

###Dependencies

install the download the lastest Eigen library from  
`http://eigen.tuxfamily.org/index.php?title=Main_Page`

###Compile From Source

From the project's root directoy create a build folder
```
mkdir build
cd build
``` 

Next run CMake to generate build scripts
```
cmake -DBUILD_SHARED_LIBS=YES ../src
```
To instead place build outputs into projects directory
```
cmake -DBUILD_SHARED_LIBS=YES -DCMAKE_INSTALL_PREFIX="${PWD}/../" ../src
```

Finally make
```
make
make install
```

###Develop with QT IDE

The following steps detail how to develop with Qt on the odroid and how to build install nessessary components to build with qmake. Throughout these instructions %VERSION% will refer to the version of Qt installing, for me it was 5.7.0, but it may be different depending on your preference.

####Install Qt

Install Qt onto the host machine by downloading Qt source from  
`https://www.qt.io/download-open-source/`

The following instructions are based on Qt's own instructions: [http://doc.qt.io/qt-5/linux-building.html](http://doc.qt.io/qt-5/linux-building.html)
To build Qt Tool chain navigate a shell to the unziped folder and run
```
./configure
make
make install
```
Next add 
```
PATH=/usr/local/Qt-%VERSION%/bin
export PATH
```
to `.profile`, after this change log-off/log-on would be required.

####Install QtCreator

To add qt creator use apt-get
```
apt-get install qtcreator
```

Something is apparently wrong with Welcome screen. Start QtCreator with no Welcome screen
```
qtcreator -noload Welcome
```
This change can be perminatly set by navigting to Help->Plugins and unchecking "Welcome"

####Configure QtCreator

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
Under "Project" option on right you may want to check "Shallow build" this helps keep your source directory clean of build artifacts.
