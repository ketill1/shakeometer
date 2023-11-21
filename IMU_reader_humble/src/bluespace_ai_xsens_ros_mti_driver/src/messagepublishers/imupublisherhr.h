// BSD 3-Clause License
//
// Copyright (c) 2021, BlueSpace.ai, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//  Copyright (c) 2003-2020 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef IMUPUBLISHERHR_H
#define IMUPUBLISHERHR_H

#include "packetcallback.h"
#include "publisherhelperfunctions.h"
#include <sensor_msgs/msg/imu.hpp>


struct ImuPublisherHR : public PacketCallback, PublisherHelperFunctions
{
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;
    double linear_acceleration_variance[3];
    double angular_velocity_variance[3];
    rclcpp::Node& node_handle;

    ImuPublisherHR(rclcpp::Node &node)
        : node_handle(node)
    {
        std::vector<double> variance = {0, 0, 0};
        node.declare_parameter("angular_velocity_stddev_hr", variance);
        node.declare_parameter("linear_acceleration_stddev_hr", variance);

        rclcpp::SensorDataQoS qos_profile;
        pub = node.create_publisher<sensor_msgs::msg::Imu>("/imu/data/hr", qos_profile);

        // REP 145: Conventions for IMU Sensor Drivers (http://www.ros.org/reps/rep-0145.html)
        // variance_from_stddev_param("angular_velocity_stddev", angular_velocity_variance, node);
        // variance_from_stddev_param("linear_acceleration_stddev", linear_acceleration_variance, node);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        bool gyro_available = packet.containsRateOfTurnHR();
        bool accel_available = packet.containsAccelerationHR();

        geometry_msgs::msg::Vector3 gyro;
        if (gyro_available)
        {
            XsVector g = packet.rateOfTurnHR();
            gyro.x = g[0];
            gyro.y = g[1];
            gyro.z = g[2];
        } else {
            gyro.x = std::nan("");
        }

        geometry_msgs::msg::Vector3 accel;
        if (accel_available)
        {
            XsVector a = packet.accelerationHR();
            accel.x = a[0];
            accel.y = a[1];
            accel.z = a[2];
        } else {
            accel.x = std::nan("");
        }

        // Imu message, publish if any of the fields is available
        if (accel_available || gyro_available)
        {
            sensor_msgs::msg::Imu msg;

            std::string frame_id = DEFAULT_FRAME_ID;
            node_handle.get_parameter("frame_id", frame_id);

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;

            msg.orientation_covariance[0] = -1;

            msg.angular_velocity = gyro;
            if (gyro_available)
            {
                msg.angular_velocity_covariance[0] = angular_velocity_variance[0];
                msg.angular_velocity_covariance[4] = angular_velocity_variance[1];
                msg.angular_velocity_covariance[8] = angular_velocity_variance[2];
            }
            else
            {
                msg.angular_velocity_covariance[0] = -1; // mark as not available
            }

            msg.linear_acceleration = accel;
            if (accel_available)
            {
                msg.linear_acceleration_covariance[0] = linear_acceleration_variance[0];
                msg.linear_acceleration_covariance[4] = linear_acceleration_variance[1];
                msg.linear_acceleration_covariance[8] = linear_acceleration_variance[2];
            }
            else
            {
                msg.linear_acceleration_covariance[0] = -1; // mark as not available
            }

            pub->publish(msg);
        }
    }
};

#endif
