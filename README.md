# IMU Unit ROS Node
This code is a slight modification of Romain Reigner's ROS node for use with RTIMU Lib, which can be found here: https://github.com/romainreignier/rtimulib_ros
This modified code is desgined to be used by Nova Rover for URC 2018.

The IMU unit being used: http://www.robotshop.com/en/imu-10-dof-16g-3-axis-accelerometer-2000--s-gyromagnetometerbarometer-c.html

## Dependencies:

- RTIMULib2 (https://github.com/RTIMULib/RTIMULib2)
- libRTIMULib.so must be copied and pasted from the include/RTIMULib/build directory into <catkin workspace>/devel/lib


## Physical Connections:

The module should be connected to the Raspberry Pi as such:

  IMU Module:     Raspberry Pi:
  * Vcc        -->  3.3V
  * GND        -->  Ground
  * SDA         -->  SDA1
  * SCL         -->  SCL1

## Publications:

Topic:       **/rtimulib_node/imu**<br />
Msg type:    sensor_msgs/imu<br />
Description: Outputs orientation, angular velocity (from gyro) and linear acceleration in the x, y and z directions.
