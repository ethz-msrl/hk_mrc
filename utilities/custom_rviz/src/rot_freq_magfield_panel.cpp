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

#include <math.h>
#include <string>

#include "rot_freq_magfield_panel.h"

#include <mag_msgs/FieldStamped.h>
#include <ros/subscriber.h>
#include <rosgraph_msgs/Log.h>

#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QVector>

namespace navion {

RotFreqMagFieldPanel::RotFreqMagFieldPanel(QWidget *parent)
    : rviz::Panel(parent),
      //magfield_topic_("/navion/desired_field_rot_axis"),
      rotfreq_topic_("/navion/desired_field_rot_frequency") {
  layout_ = new QVBoxLayout;

  // magfield_topic_edit_ = new QLineEdit(magfield_topic_);
  rotfreq_topic_edit_ = new QLineEdit(rotfreq_topic_);
  // frame_edit_ = new QLineEdit("mns");
  // magnitude_slider_ = new QSpinBox();
  // magnitude_slider_->setRange(0, 10);
  // magnitude_slider_->setSingleStep(1);
  // posx_sb_ = new QSpinBox();
  // posx_sb_->setRange(-150, 150);
  // posx_sb_->setSingleStep(5);
  // posy_sb_ = new QSpinBox();
  // posy_sb_->setRange(-150, 150);
  // posy_sb_->setSingleStep(5);
  // posz_sb_ = new QSpinBox();
  // posz_sb_->setRange(-150, 150);
  // posz_sb_->setSingleStep(5);
  // inclination_angle_sb_ = new QSpinBox();
  // inclination_angle_sb_->setRange(-180, 180);
  // inclination_angle_sb_->setSingleStep(5);
  // inclination_angle_sb_->setWrapping(true);
  // azimuth_angle_sb_ = new QSpinBox();
  // azimuth_angle_sb_->setRange(-180, 180);
  // azimuth_angle_sb_->setSingleStep(5);
  // azimuth_angle_sb_->setWrapping(true);
  rot_frequency_sb_ = new QSpinBox();
  rot_frequency_sb_->setRange(0, 15);
  rot_frequency_sb_->setSingleStep(1);
  rot_frequency_sb_->setWrapping(false);

  QFormLayout *formLayout = new QFormLayout;
  // formLayout->addRow(tr("Magnetic Field Topic:"), magfield_topic_edit_);
  formLayout->addRow(tr("Rotation Frequency Topic:"), rotfreq_topic_edit_);
  // formLayout->addRow(tr("Frame:"), frame_edit_);
  // formLayout->addRow(tr("Pos. x (mm):"), posx_sb_);
  // formLayout->addRow(tr("Pos. y (mm):"), posy_sb_);
  // formLayout->addRow(tr("Pos. z (mm):"), posz_sb_);
  // formLayout->addRow(tr("Magnitude (mT):"), magnitude_slider_);
  // formLayout->addRow(tr("Inclination Angle (deg):"), inclination_angle_sb_);
  // formLayout->addRow(tr("Azimuth Angle (deg):"), azimuth_angle_sb_);
  formLayout->addRow(tr("Rotation Frequency (Hz):"), rot_frequency_sb_);
  layout_->addLayout(formLayout);

  // connect(magfield_topic_edit_, SIGNAL(editingFinished()), this, SLOT(updateFieldTopic()));
  connect(rotfreq_topic_edit_, SIGNAL(editingFinished()), this, SLOT(updateFreqTopic()));
  // connect(magnitude_slider_, SIGNAL(valueChanged(int)), this, SLOT(updateMagVector(int)));
  // connect(azimuth_angle_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateMagVector(int)));
  // connect(inclination_angle_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateMagVector(int)));
  // connect(posx_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateMagVector(int)));
  // connect(posy_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateMagVector(int)));
  // connect(posz_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateMagVector(int)));
  connect(rot_frequency_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateRotationFrequency(int)));

  setLayout(layout_);
  // mag_field_pub_ = nh_.advertise<mag_msgs::FieldStamped>(magfield_topic_.toStdString(), 1000);
  rot_freq_pub_ = nh_.advertise<std_msgs::Float32>(rotfreq_topic_.toStdString(), 1000);

  // updateMagVector(0);
  updateRotationFrequency(0);
}

void RotFreqMagFieldPanel::load(const rviz::Config &config) {
  rviz::Panel::load(config);
  // QString magfield_topic;
  // if (config.mapGetString("magfield_topic", &magfield_topic)) {
  //   magfield_topic_edit_->setText(magfield_topic);
  //   updateFieldTopic();
  // }
  QString rotfreq_topic;
  if (config.mapGetString("rotfreq_topic_", &rotfreq_topic)) {
    rotfreq_topic_edit_->setText(rotfreq_topic);
    updateFreqTopic();
  }
  // QString frame_id;
  // if (config.mapGetString("frame_id", &frame_id)) {
  //   frame_edit_->setText(frame_id);
  // }
  // updateMagVector(0);
  updateRotationFrequency(0);
}

void RotFreqMagFieldPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  // config.mapSetValue("magfield_topic", magfield_topic_);
  config.mapSetValue("rotfreq_topic_", rotfreq_topic_);
  // config.mapSetValue("frame_id", frame_edit_->text());
}

// void RotFreqMagFieldPanel::updateFieldTopic() { setFieldTopic(magfield_topic_edit_->text()); }

// void RotFreqMagFieldPanel::setFieldTopic(const QString &topic_name) {
//   if (topic_name == "") {
//     ROS_DEBUG("Empty topic");
//     return;
//   }

//   if (topic_name == magfield_topic_) {
//     ROS_DEBUG("Already subscribed to topic");
//   }

//   magfield_topic_ = topic_name;
//   mag_field_pub_.shutdown();
//   mag_field_pub_ = nh_.advertise<mag_msgs::FieldStamped>(magfield_topic_.toStdString(), 1000);

//   Q_EMIT configChanged();
// }

void RotFreqMagFieldPanel::updateFreqTopic() { setFreqTopic(rotfreq_topic_edit_->text()); }

void RotFreqMagFieldPanel::setFreqTopic(const QString &topic_name) {
  if (topic_name == "") {
    ROS_DEBUG("Empty topic");
    return;
  }

  if (topic_name == rotfreq_topic_) {
    ROS_DEBUG("Already subscribed to topic");
  }

  rotfreq_topic_ = topic_name;
  rot_freq_pub_.shutdown();
  rot_freq_pub_ = nh_.advertise<std_msgs::Float32>(rotfreq_topic_.toStdString(), 1000);

  Q_EMIT configChanged();
}

// void RotFreqMagFieldPanel::updateMagVector(int value) {
//   mag_msgs::FieldStamped mf_msg;
//   mf_msg.header.stamp = ros::Time::now();
//   mf_msg.header.frame_id = frame_edit_->text().toStdString();

//   double pi = std::acos(-1);
//   double r = static_cast<double>(magnitude_slider_->value());
//   double ia = static_cast<double>(inclination_angle_sb_->value()) / 180 * pi;
//   double aa = static_cast<double>(azimuth_angle_sb_->value()) / 180 * pi;
//   double posx = static_cast<double>(posx_sb_->value()) / 1000.;
//   double posy = static_cast<double>(posy_sb_->value()) / 1000.;
//   double posz = static_cast<double>(posz_sb_->value()) / 1000.;

//   mf_msg.field.vector.x = r * sin(ia) * cos(aa) * 0.001;
//   mf_msg.field.vector.y = r * sin(ia) * sin(aa) * 0.001;
//   mf_msg.field.vector.z = r * cos(ia) * 0.001;

//   mf_msg.field.position.x = posx;
//   mf_msg.field.position.y = posy;
//   mf_msg.field.position.z = posz;

//   mag_field_pub_.publish(mf_msg);
// }

void RotFreqMagFieldPanel::updateRotationFrequency(int value) {
  std_msgs::Float32 freq_msg;

  freq_msg.data = static_cast<float>(rot_frequency_sb_->value());
  rot_freq_pub_.publish(freq_msg);
}

}  // namespace navion

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(navion::RotFreqMagFieldPanel, rviz::Panel)
