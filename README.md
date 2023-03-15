# PULSAR (powered-flying ultra-underactuated LiDAR sensing aerial robot)

## A self-rotating, single-actuated UAV with extended sensor field of view for autonomous navigation

**Paper Link**: https://www.science.org/doi/epdf/10.1126/scirobotics.ade4538

Click for the overview video.

<!-- [![Video Demo](./img/out.png)](https://www.youtube.com/watch?v=eDkwGXCea7w) -->

## 1 Structure CAD files

Including the CAD files of PULSAR and the swashplateless mechanism.

## 2 Hardware deisgn files

Including the hardware design files of the magnetic encoder.

## 3 PX4 firmware of PULSAR

### 3.1 Hardware configuration
+ Power the magnetic encoder and config it using a I2C bus, and make sure the 910-Hz PWM signal is outputed (e.g., verified by a oscilloscope).
+ Connect the power port of magnetic encoder to a 5V port of the Pixhawk 4 Mini.
+ Connect the PWM signal of magnetic encoder to the CAP1 port of the Pixhawk 4 Mini.
+ Connect the ESC (off-the-shelf ESC is ok as long as it supports dshot protocol) to the MAIN OUT 1 port of the Pixhawk 4 Mini.

### 3.2 Software configuration
+ Clone the repository
+ Config your compile environment https://docs.px4.io/main/en/dev_setup/dev_env.html
+ Build the code and upload to Pixhawk 4 mini through USB connection
```
cd PULSAR
git submodule update --init
cd 3-PX4_Firmware/PULSAR_PX4
make px4_fmu-v5_default upload
```
+ Load the parameter file "PULSAR_PX4_parameters.params" to the Pixhawk 4 mini by QGroundControl or other tools (very important)
+ Keeping the USB connection, in the MAVLINK Console of QGroundControl, type
```
px4_cn start
```
+ If the encoder works well, two rotor angles (angle_compensated and angle_raw) will be printed in the MAVLINK terminal. Rotate the positive blade of the swashplateless mechanism to align with the negative direction of the body y axis of PULSAR (facing to LiDAR's front side, right hand side is the nevative y axis), and record the value of the printed "angle_raw". 
+ Modified the MACRO (SENSOR_ROTOR_ANGLE_BIAS) with the recorded value (see 3.2.2).
+ Compile the code and upload again.
+ Keeping the same blade's position and starting the angle print in MAVLINK again, make sure the "angle_compensated" is close to 0 deg or 360 deg.
+ To stop the print, type
```
px4_cn stop
```


### 3.3 Main modifications of the PX4 firmware

#### 3.3.1 PWM capture driver for magnetic encoder 
The files are located in "src/drivers/pwm_input". Please make sure that the encoder 910-Hz PWM signal is connected to the CAP1 of the Pixhawk 4 Mini. Changing the frequency should also modify the timer frequency in "PULSAR_PX4/src/drivers/pwm_input/pwm_input.cpp".

#### 3.3.2 Mixer of PULSAR
The mixer files are located in the "PULSAR_PX4/src/drivers/dshot/swashplateless".

SENSOR_ROTOR_ANGLE_BIAS (unit is degree) is used for compensating the encoder installation offset angle and it must be set properly.

MOTOR_DELAY_ANLGE_BIAS (unit is radian) is used for compensating the lag angle of the swashplateless mechanism. This value is different when using different motor or propeller and can be determined on a test stand with 6-axis force sensor by analyzing the moment data. If you use the propulsion system same as PULSAR (i.e., T-MOTOR MN5006 KV450 and MF1302 propeller), you don't need to change this value.

The data of swashplateless mechanism are published for logging.


#### 3.3.3 The repalcement of throttle output
The repalcement is conducted in "PULSAR_PX4/src/drivers/dshot/dshot.cpp".
Please make sure that the ESC signal wire is connected to the MAIN OUT1 of the Pixhawk 4 Mini. The dshot has been enabled if you had loaded the parameter file into the Pixhawk 4 mini.


#### 3.3.4 The attitude controller of PULSAR
The attitude controller is implemented in "PULSAR_PX4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp".
The attitude setpoint and feedback are published for data logging.


#### 3.3.5 The angular velocity controller of PULSAR
The angular velocity controller is implemented in "PULSAR_PX4/src/modules/sa_rate_control_me/sa_rate_control_me_main.cpp".
The angular velocity setpoint and feedback both in world frame are published for data logging.


#### 3.3.6 The position controller of PULSAR
There are some modifications in "PULSAR_PX4/src/modules/mc_pos_control" and "PULSAR_PX4/src/lib/flight_tasks".
This ensure the position command from remote controller (RC) is available in world frame rather than in body frame.

#### 3.3.7 Logging topics
Some ulog topics are added in "PULSAR_PX4/src/modules/logger/logged_topics.cpp" and "PULSAR_PX4/msg".

## 4 Software modules

### 4.1 FAST-LIO2 
Different from the standard version of FAST-LIO2, the FAST-LIO2 in this repository contains coordinate transformations of both odometry and point clouds because the LiDAR (Livox AVIA) is installed on PULSAR with a 90-degree rotated orientation. Refer to https://github.com/hku-mars/FAST_LIO for more configuration details.

To transmit the odometry from FAST-LIO2 to PX4, [MAVROS](https://github.com/mavlink/mavros) should be run with FAST-LIO2 at the same time
```
roslaunch mavros px4.launch
roslaunch fastlio mapping_avia_indoor.launch  # indoor environment
roslaunch fastlio mapping_avia_outdoor.launch  # outdoor environment
```

In addition, [Point-LIO](https://github.com/hku-mars/Point-LIO) is more suitable for PULSAR since it can handle higher self-rotation rate.


### 4.2 Planner
The new version of the planner used in PULSAR is in https://github.com/hku-mars/dyn_small_obs_avoidance.
Other planners can also be used for PULSAR if they can receive the odometry and point clouds provided by FAST-LIO2, Point-LIO, or other LIOs.


### 4.3 Incremental kd-forest
The incremental kd-forest is a data strcuture which contains many [ikd-trees](https://github.com/hku-mars/ikd-Tree).
You may contact [Yixi Cai](https://github.com/Ecstasy-EC) if there is any issue.


