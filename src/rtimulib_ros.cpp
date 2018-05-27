// RTIMULib ROS Node
// Copyright (c) 2017, Romain Reignier
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the <organization> nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <RTIMULib.h>
#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int32.h>

#define LOOP_HERTZ 10
#define FILTER_WINDOW_SIZE 5

int window[FILTER_WINDOW_SIZE];
int i = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtimulib_node");
    ROS_INFO("Imu driver is now running");
    ros::NodeHandle nh ("/");
    ros::Rate loop_rate(LOOP_HERTZ);

    std::string calibration_file_path;
    calibration_file_path = "/home/nova/catkin_ws/src/rtimulib_ros/config";

    std::string calibration_file_name = "RTIMULib";

    std::string frame_id = "imu_link";

    ros::Publisher bearingPub = nh.advertise<std_msgs::Int32>("/bearing",1);

    // Load the RTIMULib.ini config file
    RTIMUSettings *settings = new RTIMUSettings(calibration_file_path.c_str(),
                                                calibration_file_name.c_str());

    RTIMU *imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
    {
        ROS_ERROR("No Imu found");
        ROS_BREAK();
    }

    // Initialise the imu object
    imu->IMUInit();

    // Set the Fusion coefficient
    //imu->setSlerpPower(0.02);
    // Enable the sensors
    //imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    while (ros::ok())
    {
        if (imu->IMURead())
        {
            RTIMU_DATA imu_data = imu->getIMUData();

            std_msgs::Int32 bearing;

            float accel_x = imu_data.accel.x();
            float accel_y = imu_data.accel.y();
            float mag_x = imu_data.compass.x();
            float mag_y = imu_data.compass.y();
            float centre_x = -53.5778017315*accel_x + 30.1703329121;
            float centre_y = -43.1731374079*accel_y + 6.4688462339;

            //ROS_INFO("%f %f", mag_x, mag_y);
            float bearing_radian = 1.5*M_PI - atan2(mag_y-centre_y,mag_x-centre_x);
            int bearing_degree = int(bearing_radian*180/M_PI) % 360;

            window[i++] = bearing_degree;
            if (i == FILTER_WINDOW_SIZE) i=0;

            int sum = 0;
            for (int j = 0; j < FILTER_WINDOW_SIZE; j++) sum += window[j];

            bearing.data = sum / FILTER_WINDOW_SIZE;
            //bearing.data = bearing_degree;
            bearingPub.publish(bearing);

            /*
            imu_msg.orientation.x = imu_data.fusionQPose.x(); 
            imu_msg.orientation.y = imu_data.fusionQPose.y(); 
            imu_msg.orientation.z = imu_data.fusionQPose.z(); 
            imu_msg.orientation.w = imu_data.fusionQPose.scalar(); 

            imu_msg.angular_velocity.x = imu_data.gyro.x();
            imu_msg.angular_velocity.y = imu_data.gyro.y();
            imu_msg.angular_velocity.z = imu_data.gyro.z();

            imu_msg.linear_acceleration.x = imu_data.accel.x() * G_TO_MPSS;
            imu_msg.linear_acceleration.y = imu_data.accel.y() * G_TO_MPSS;
            imu_msg.linear_acceleration.z = imu_data.accel.z() * G_TO_MPSS;
            */
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
