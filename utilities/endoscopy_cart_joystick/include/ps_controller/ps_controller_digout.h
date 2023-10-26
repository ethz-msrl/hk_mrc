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
// #include <std_msgs/Float32.h>
// #include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>
// #include <std_srvs/Trigger.h>
#include <endoscopy_cart_controller/digital_outputs.h>
#include <endoscopy_cart_controller/setDigout.h>



class PsNaviController {

public:

    // default ctor
    PsNaviController(ros::NodeHandle& nh);

    // dtor
    ~PsNaviController();

    // Subcriber callback
    void processJoyMsgs(const sensor_msgs::Joy& msg);


private:
    // endoscopy_cart_controller::digital_outputs dig_out_msg_;

    // ros::Publisher dig_out_pub_;
    ros::Subscriber ps_navi_sub_;
    ros::ServiceClient setDigout_srv;
 
    int axis_lr_;
    int axis_ud_;
    bool digout_state[8];
    bool digout_state_prev[8];

};


#endif // PS_NAVI_CONTROLLER_H
