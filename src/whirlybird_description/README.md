## Synopsis

This package contains the necessary rviz and urdf files to run a rviz visualization of the whirlybird hardware.
Currently, this package assumes that you are running Ubuntu 16.04 with the ros-kinetic toolchain. Note: this 
package will not run on Ubuntu 14.04 with Indigo due to Qt4 vs Qt5 discrepancies.

## Example

To run the visualization execute the following lines of code:
```
cd <path to catkin_ws>
catkin_make
source devel/setup.bash
roslaunch whirlybird_description visualize.launch
```

This visualization contains sliders for Pitch and Yaw, which publish to the topics `\theta_r` and `\phi_r`. In turn,
these will will need to be fed into a controller to send the proper PWM commands to the whirlybird. There is also a 
visualization that interacts directly with the PWM commands, this is to be used only to calibrate the PWM gain on 
the whirlybird hardware. The PWM visualization can be launched with 
```
roslaunch whirlybird_description visualize.launch
```

Note: these commands will only bring up rviz and thus, the whirlybird dynamics will not be implemented. To implement the
whirlybird dynamics, either the simulator or the hardware needs to be run. More details are found in the 
`whirlybird_sim` and `whirlybird_serial` packages.
