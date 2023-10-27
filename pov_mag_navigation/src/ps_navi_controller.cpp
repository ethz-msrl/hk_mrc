//
// Tesla - A ROS-based framework for performing magnetic manipulation
//
// Copyright 2018 Multi Scale Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include "ps_navi_controller/ps_navi_controller.h"

#include <stdio.h>
#include <string.h>
#include <tf/transform_datatypes.h>

PsNaviController::PsNaviController(ros::NodeHandle& nh):
    joy_axis_0_(1),
    joy_axis_1_(0),
    u_des_(0.),
    v_des_(0.),
    gain_(1.),
    joy_axis_l3_(3),
    button_up_(8),
    button_down_(9),
    button_l1_(5),
    button_l2_(4),
    button_l3_(7),
    button_o_(1),
    button_x_(0),
    button_ps_(6)
{
    ROS_INFO("Starting PS navi controller");

    ros::NodeHandle nhp("~");
    nhp.param<int>("joy_axis_0", joy_axis_0_, joy_axis_0_);
    nhp.param<int>("joy_axis_1", joy_axis_1_, joy_axis_1_);
    nhp.param<int>("button_up", button_up_, button_up_);
    nhp.param<int>("button_down", button_down_, button_down_);
    nhp.param<int>("button_l1", button_l1_, button_l1_);
    nhp.param<int>("button_l2", button_l2_, button_l2_);
    nhp.param<int>("button_l3", button_l3_, button_l3_);
    nhp.param<int>("button_o", button_o_, button_o_);
    nhp.param<int>("button_x", button_x_, button_x_);



    // Define subscriber and publisher
    ps_navi_sub_ = nh.subscribe("/joy", 1, &PsNaviController::processJoyMsgs, this);
    target_speed_pub_ = nh.advertise<opencv_apps::Point2D>("/speed_des",1);
    change_field_mag_pub_ = nh.advertise<std_msgs::Int32>("/change_field_mag",1);
    insertion_pub_ = nh.advertise<std_msgs::Float32>("/insert_speed",1);
    calib_client_ = nh.serviceClient<std_srvs::Trigger>("/do_calib");
    save_config_client_ = nh.serviceClient<std_srvs::Trigger>("/save_config");
    goto_config_client_ = nh.serviceClient<std_srvs::Trigger>("/goto_config");

    flag_button_up_ = true;
    flag_button_down_ = true;
    
    insert_speed_gain_ = 0.0001;

}

PsNaviController::~PsNaviController()
{
}


void PsNaviController::processJoyMsgs(const sensor_msgs::Joy& msg)
{
    // //Call calibration
    // if(msg.buttons.at(button_l3_)){
    //     ROS_INFO("Calib");
    //     std_srvs::Trigger srv;
    //     if(calib_client_.call(srv)) {
    //         ROS_INFO_STREAM("Service success: " << srv.response.message);
    //         } else {
    //             ROS_WARN_STREAM("Reset service call failed.");
    //         }
    // }

    // //Call save_config
    // if(msg.buttons.at(button_o_)){
    //     ROS_INFO("Save config");
    //     std_srvs::Trigger srv;
    //     if(save_config_client_.call(srv)) {
    //         ROS_INFO_STREAM("Service success: " << srv.response.message);
    //         } else {
    //             ROS_WARN_STREAM("Reset service call failed.");
    //         }
    // }

    // //Call goto config
    // if(msg.buttons.at(button_x_)){
    //     ROS_INFO("Go to saved config");
    //     std_srvs::Trigger srv;
    //     if(goto_config_client_.call(srv)) {
    //         ROS_INFO_STREAM("Service success: " << srv.response.message);
    //         } else {
    //             ROS_WARN_STREAM("Reset service call failed.");
    //         }
    // }

    //Change field mag
    if(msg.buttons.at(button_up_)){
        if(flag_button_up_){
            ROS_INFO("Request increase field mag");
            std_msgs::Int32 delta;
            delta.data = 1;

            //Publish desired speed
            change_field_mag_pub_.publish(delta);

            flag_button_up_ = false;
        }
    } else {
        flag_button_up_ = true;
    }


    if(msg.buttons.at(button_down_)){
        if(flag_button_down_){
            ROS_INFO("Request decrease field mag");
            flag_button_down_ = false;
            std_msgs::Int32 delta;
            delta.data = -1;

            //Publish desired speed
            change_field_mag_pub_.publish(delta);

            flag_button_down_ = false;
        }
    } else {
        flag_button_down_ = true;
    }


    double v_des = gain_ * msg.axes.at(joy_axis_0_); // px/s
    double u_des = -gain_ * msg.axes.at(joy_axis_1_); // px/s

    // Publish target speed in image
    opencv_apps::Point2D target;
    target.x = u_des;
    target.y = v_des;

    //Publish desired speed
    target_speed_pub_.publish(target);

    // insertion
    std_msgs::Float32 insert_speed;
    if(msg.buttons.at(button_l1_)){
        insert_speed.data = 1.*insert_speed_gain_;
    }
    if(msg.buttons.at(button_l2_)){
        insert_speed.data = -1.*insert_speed_gain_;
    }
    insertion_pub_.publish(insert_speed);


}


// Standard C++ entry point
int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "ps_navi_controller");
  ros::NodeHandle nh("ps_navi_controller");

  PsNaviController apc(nh);

  ros::Rate loop_rate(30);
  while (ros::ok())
  {

      ros::spinOnce();
      loop_rate.sleep();
  }
  
  return 0;
}
