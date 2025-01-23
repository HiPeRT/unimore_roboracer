# adx_data
This library implements a set of custom data types that can replace ROS data types. The goal is independence from ROS and support with external high performance libraries, one over all [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page).

Linear algebra is the basic building block of most robotics software. Being able to use high-performance libraries ensures higher ease of use, improves code readability and optimization.

## Dependencies
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)

## Available data types

### Vector
Representation of a vector defined using `Eigen::Matrix<Type, Size, 1>`.

----

### Point

This contains the position of a point in free space.

----

### TimeStamp
Generic data header with timestamp and frame_id. This header stores a timestamp of the received data and a frame_id to identify in which set of coordinates a specific message resides. An exact representation of time, stored as a `chrono::time_point`.

----

### Header
This header stores a timestamp of the received data and a frame_id to identify in which set of coordinates a specific message resides.


----

### Quaternion

This represents an orientation in free space in quaternion form. In this case, the data type inherits from the Quaternion data type defined by eigen `Eigen::Quaternion<Type>`.

----

### Covariance
Size x Size covariance matrix. Size is the number of values that may be correlated, using an Eigen matrix as the underlying data type. The struct inherits from eigen matrix `Eigen::Matrix<Type, Size, Size>`.

----

### Twist
This represents velocity in free space broken into its linear and angular components. The twist is defined by two `adx::data::Vector3d` representing the linear velocity component and the angular velocity and one `Covarianced<6>` in order there should be x-linear, y-linear, z-linear, x-angular, y-angular, z-angular.

----

### Pose

The representation of a pose, with position and orientation of the robot. Can optionally include headers and covariances

----

### Particle

A weighted estimate of robot pose, pose represents the estimated position and orientation of the robot.
Weight represents the estimated weight of this particle. This data type is derived from Pose and Weight.

----

### Odometry

Odometry is an estimate of a position and velocity in free space. Pose represents the estimated position and orientation of the robot. Twist represents the estimated velocities of the robot. The odometry data type inherits from Header, Pose3d and Twist.

----

### Obstacle

This struct represents a cube obstacle. The obstacle data type inherits from Pose and Twist. The cube half-lenghts, the linear combination of position and size gives us the cube vertices for this I define an `adx::data::Vector3d`.

---

### Plan

This is essentially just a Path that uses Point instead of Pose and stores a reference speed for each point This is useful in racing environments. The plane data type inherits from header. For the positions are used a series of points that are used to represent the path `std::vector<Point3d>`. Speeds are represented by a series of adx Vector3d `std::vector<adx::data::Vector3d>`.

---

### Path

A series of Pose representing a Path for robot navigation. The path data type inherits from header. For the rappresentation of the path are used a series of poses `std::vector<Pose<double, 3, false, false>>`.

## Authors
* **Michele Guzzinati** - [mguzzina](https://github.com/mguzzina)
* **Federico Gavioli** - [fgavioli](https://github.com/fgavioli) - _(Maintainer)_
* **Andrea Bernardi** - [andreabernard](https://github.com/andreabernard)

## AD Project
This repository is part of the autonomous driving project of the University of Modena and Reggio Emilia, [read more](https://hipert.github.io/ad_site/).

## License
**Apache 2.0** - [License](https://opensource.org/licenses/Apache-2.0)
