
#Building

Use information bellow to buld entire source tree for your platform

##Requirments
List which MACE has been successfully targeted.
* GCC 4.6.4+ OR MSVC2013
* QT 5.7.0+


## Dependencies

### Eigen

Download and install the lastest Eigen library from  
`http://eigen.tuxfamily.org/index.php?title=Main_Page`

For both Unix/Windows the default install location is fine.
For windows you may need to open a `cmd.exe` window with adminstator privaliges, then run `INSTALL.vcxproj` in the build directory such that the "INSTALL" project can copy it's files onto the filesystem.

### Download Mavlink libraries

The Mavlink library is configured as a submodule in git.
To download the nessessary files, run the following command in a shell on the project.
```
git submodule update --init --recursive
```

### Qt Libraries

Currently we utialize QtSerialPort library to faciliate communication to vehicles therefore Qt's libraries are required. However it is not required to use Qt's toolchain to build/develop MACE.

For Windows or "typical" linux install QT as per normal

#### Building On ODROID

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

##Command Line Compile MACE with UNIX

###CMake

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

###QMake

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

#Develop with QT IDE on ODROID

The following steps detail how to develop with Qt on the odroid and how to install and operate qtcreator on the odroid to build MACE.

##Install QtCreator

To add qt creator use apt-get
```
apt-get install qtcreator
```

Something is apparently wrong with Welcome screen. Start QtCreator with no Welcome screen
```
qtcreator -noload Welcome
```
This change can be perminatly set by navigting to Help->Plugins and unchecking "Welcome"

##Configure QtCreator

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

#Develop with QT IDE on Windows

The following steps detail how to develop with Qt on windows.

##Install QtCreator

Likely this was installed in the above step to 

##Configure QtCreator

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
