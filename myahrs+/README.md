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

cant change : void imu_feedback(10)    to    std::vector<double> imu_feedback(10) to benefit of the return variable ?!!!