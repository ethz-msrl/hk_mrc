//
// Tesla - A ROS-based framework for performing magnetic manipulation
//
// Software License Agreement (BSD License)
//
// ©2022 ETH Zurich, D-​MAVT; Multi-Scale Robotics Lab (MSRL) ; Prof Bradley J. Nelson
// All rights reserved.
//
// Redistribution and use of this software in source and binary forms,
// with or without modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above
//   copyright notice, this list of conditions and the
//   following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * All advertising materials mentioning features or use of this software
//   must display the following acknowledgement:
//   “This product includes software developed by the Multi-Scale Robotics Lab, ETH Zurich,
//   Switzerland and its contributors.”
//
// * Neither the name of MSRL nor the names of its
//   contributors may be used to endorse or promote products
//   derived from this software without specific prior
//   written permission of MSRL.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#ifndef Q_MOC_RUN
#include <mag_msgs/FieldStamped.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <rosgraph_msgs/Log.h>
#include <rviz/panel.h>
#include <std_msgs/Float32.h>
#endif

class QLineEdit;
class QLabel;
class QVBoxLayout;
class QGridLayout;
class QSlider;
class QSpinBox;

namespace navion {

class RotFreqMagFieldPanel : public rviz::Panel {
  Q_OBJECT

 public:
  explicit RotFreqMagFieldPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

 public Q_SLOTS:

  //void setFieldTopic(const QString& topic_name);

  void setFreqTopic(const QString& topic_name);

 protected Q_SLOTS:

  //void updateFieldTopic();

  void updateFreqTopic();

  // void updateMagVector(int value);

  void updateRotationFrequency(int value);

 protected:
  QLineEdit* magfield_topic_edit_;
  QLineEdit* rotfreq_topic_edit_;
  // QLineEdit* frame_edit_;
  // QString magfield_topic_;
  QString rotfreq_topic_;
  // QSpinBox* magnitude_slider_;
  // QSpinBox* inclination_angle_sb_;
  // QSpinBox* azimuth_angle_sb_;
  QSpinBox* rot_frequency_sb_;
  // QSpinBox* posx_sb_;
  // QSpinBox* posy_sb_;
  // QSpinBox* posz_sb_;
  //ros::Publisher mag_field_pub_;
  ros::Publisher rot_freq_pub_;

  QVBoxLayout* layout_;

  ros::NodeHandle nh_;
};
}  // namespace navion
