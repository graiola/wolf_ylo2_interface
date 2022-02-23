# Directory Test about Myahrs, using C++ Code

* makefile tells how to compile

* myahrs_plus_hpp is the library header

* the *.cpp file is the program to compile

# To compile

* go under correct Dir,

* type : make all

* run : ./elpimous_test_myahrs+  ( read values)

## comments :

Code working :

imu initializes correctly, sends quaternion, linear and angular.

	can printf values when feeded in float imu_feedback(10)

	only return series of 0000 when feeded in std::vector<double> imu_feedback(10)
