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

Install QT onto the host machine by downloading QT source from  
`https://www.qt.io/download-open-source/`

You will need to download two things: the QT source and QT Creator source.

The following instructions are based on Qt's own instructions: [http://doc.qt.io/qt-5/build-sources.html](http://doc.qt.io/qt-5/build-sources.html)
To build QT Tool chain navigate a shell to the unziped folder and run
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

To build QT Creator navigate a shell to the unzipped folder and run
```

```

