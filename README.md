# modelling-control-and-simulation
C++ and Python implementations of projects inspired mainly by the courses Underactuated Robotics and Robotic Manipulation by Russ Tedrake.

[Underactuated Robotics](https://underactuated.csail.mit.edu/index.html)

[Robotic Manipulation](https://manipulation.csail.mit.edu/index.html)

# Video
Videos of some controllers in action are found in /video

# Build
```
//install drake and set the drake path in CMakeLists.tct
mkdir build; cd build
cmake ..
cmake --build . 
```
# Run
```
// Set controller config in configs/<plant>/collocated_energy_ctrl.yaml
// Set init state, simukation time etc configs/<plant>/experiment.yaml (Not implemetned for all)
cd build
./acrobot_visualization
```
