### LIDAR Obstacle Detection

### Dependencies
To install the latest version of [PCL](http://pointclouds.org/) (currently 1.9.1):

#### Ubuntu 
```bash
$ sudo apt install libflann-dev libboost-all-dev libeigen3-dev 

# build and install VTK
$ git clone --branch v8.2.0 https://github.com/Kitware/VTK.git
$ cd VTK
$ mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ../
$ make -j4
$ sudo make install

# build and install PCL
$ git clone --branch pcl-1.9.1 https://github.com/PointCloudLibrary/pcl.git
$ cd pcl
$ mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_visualization=ON ../
$ make -j4
$ sudo make install
```

#### Build and Run

```shell script
$ cd SFND_Lidar_Obstacle_Detection
$ mkdir build && cd build
$ cmake ..
$ make
$ ./environment
```
---
  
# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="350" height="200" />

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.
