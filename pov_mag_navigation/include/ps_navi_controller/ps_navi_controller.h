/* * Tesla - A ROS-based framework for performing magnetic manipulation
 *
 * Copyright 2018 Multi Scale Robotics Lab
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PS_NAVI_CONTROLLER_H
#define PS_NAVI_CONTROLLER_H

#include <string>
#include <stdexcept>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>

// msg
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <opencv_apps/Point2D.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>


class PsNaviController {

public:

    // default ctor
    PsNaviController(ros::NodeHandle& nh);

    // dtor
    ~PsNaviController();

    // Subcriber callback
    void processJoyMsgs(const sensor_msgs::Joy& msg);


private:
    ros::Subscriber ps_navi_sub_;
    ros::Publisher target_speed_pub_;
    ros::Publisher change_field_mag_pub_;
    ros::Publisher insertion_pub_;
    ros::ServiceClient calib_client_;
    ros::ServiceClient save_config_client_;
    ros::ServiceClient goto_config_client_;
 
    double gain_;
    double u_des_, v_des_;
    double insert_speed_gain_;

    int joy_axis_0_;
    int joy_axis_1_;
    int joy_axis_l3_;
    int button_up_;
    int button_down_;
    int button_l1_;
    int button_l2_;
    int button_l3_;
    int button_x_;
    int button_o_;
    int button_ps_;

    bool flag_button_up_;
    bool flag_button_down_;
};


#endif // PS_NAVI_CONTROLLER_H
