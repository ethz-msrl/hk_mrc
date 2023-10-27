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

#include "ps_controller/ps_controller_digout.h"

#include <stdio.h>
#include <string.h>
#include <tf/transform_datatypes.h>

PsNaviController::PsNaviController(ros::NodeHandle& nh):
    axis_lr_(6),
    axis_ud_(7)
{
    ROS_INFO("Starting PS controller");

    ros::NodeHandle nhp("~");
    nhp.param<int>("axis_lr", axis_lr_, axis_lr_);
    nhp.param<int>("axis_ud", axis_ud_, axis_ud_);

    // Define subscriber and publisher
    ps_navi_sub_ = nh.subscribe("/joy", 1, &PsNaviController::processJoyMsgs, this);
    // dig_out_pub_ = nh.advertise<endoscopy_cart_controller::digital_outputs>("/joy/digital_outputs_target", 1000);
    setDigout_srv = nh.serviceClient<endoscopy_cart_controller::setDigout>("/cart/digout_req");


    for(int i=0;i<8;i++){
        digout_state[i] = false;
        digout_state_prev[i] = false;
    }

}

PsNaviController::~PsNaviController()
{
}

void PsNaviController::processJoyMsgs(const sensor_msgs::Joy& msg)
{

    // // digital out
    // dig_out_msg_.stamp = ros::Time::now();

    for(int i=0;i<8;i++){
        digout_state[i] = false;
    }


    // Map arrows to first 4 digital outputs
    if(msg.axes.at(axis_lr_) > 0.5){
        digout_state[4] = true;
    }else{
        if(msg.axes.at(axis_lr_) < -0.5){
            digout_state[5] = true;
        }
    }

    if(msg.axes.at(axis_ud_) > 0.5){
        digout_state[6] = true;
    }else{
        if(msg.axes.at(axis_ud_) < -0.5){
            digout_state[7] = true;
        }
    }

    // dig_out_pub_.publish(dig_out_msg_);


    for(int i=0;i<8;i++){
        if(digout_state[i] != digout_state_prev[i]){

            ROS_INFO_STREAM("Send digout digout from joytick..." );

            endoscopy_cart_controller::setDigout srv;
            srv.request.digout_idx =  i;
            srv.request.digout_state = digout_state[i];
            setDigout_srv.call(srv);

        }
        digout_state_prev[i] = digout_state[i];
    }





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
