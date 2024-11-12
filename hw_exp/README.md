# MPC for learned quadruped model

This code is for jumping experiment in A1 robot hardware

## How to setup experiments
1. Check the IP address of Mocap: 192.168.123.88
2. Check the reading of MoCap, we can move the robot to check the signals
3. Use Ethernet port to connect the computer with both robot and MoCap
4. Check the reading of Mocap (i.e. the quaternion) when executing the standing up 
on the robot


## How to build and run the code


ping 192.168.123.10

To build

```
mkdir
cd build
cmake ..
make -j4
```

It might require lcm package, so please download and install it by following 
https://lcm-proj.github.io/build_instructions.html

Download eigen-3.3.7 and install it by following

```
mkdir build
cd build
cmake ..
make
sudo make install
```



The subfolder lcm-1.4.0 and eigen-3.3.7 are included in the folder, we can use it for convenience.

To run

```
cd Controller
sudo ./ctrl # to run
```
 
