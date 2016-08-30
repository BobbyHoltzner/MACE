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
