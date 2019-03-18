/**
 * \file whirlybird_panel.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <QLineEdit>
#include <QSlider>
#include <QSpinBox>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QTimer>

#include <std_msgs/Float32.h>
#include <whirlybird_msgs/Whirlybird.h>

#include "whirlybird_panel.h"

namespace whirlybird_description
{

WhirlybirdPanel::WhirlybirdPanel(QWidget* parent = 0) : rviz::Panel(parent),
  pitch_setpoint_rad_(0.0f),
  yaw_setpoint_rad_(0.0f),
  pitch_max_deg_(45),
  yaw_max_deg_(45)
{
  //===========================================================================
  // controls
  //===========================================================================

  // setpoints
  pitch_slider_ = new QSlider(Qt::Horizontal);
  pitch_slider_->setRange(-pitch_max_deg_, pitch_max_deg_);
  pitch_slider_->setTickPosition(QSlider::TicksBelow);
  pitch_slider_->setTickInterval(15);

  pitch_spinbox_ = new QSpinBox;
  pitch_spinbox_->setRange(-pitch_max_deg_, pitch_max_deg_);

  yaw_slider_ = new QSlider(Qt::Horizontal);
  yaw_slider_->setRange(-yaw_max_deg_, yaw_max_deg_);
  yaw_slider_->setTickPosition(QSlider::TicksBelow);
  yaw_slider_->setTickInterval(15);

  yaw_spinbox_ = new QSpinBox;
  yaw_spinbox_->setRange(-yaw_max_deg_, yaw_max_deg_);

  // topics
  pitch_topic_editor_ = new QLineEdit;
  yaw_topic_editor_ = new QLineEdit;

  // enabled
  enabled_check_box_ = new QCheckBox;

  // timer
  QTimer* setpoint_timer = new QTimer(this);

  //===========================================================================
  // layout
  //===========================================================================

  QGridLayout* setpoints_layout = new QGridLayout;
  setpoints_layout->addWidget(new QLabel("Pitch"), 0, 0);
  setpoints_layout->addWidget(pitch_slider_, 0, 1);
  setpoints_layout->addWidget(pitch_spinbox_, 0, 2);
  setpoints_layout->addWidget(new QLabel("Yaw"), 1, 0);
  setpoints_layout->addWidget(yaw_slider_, 1, 1);
  setpoints_layout->addWidget(yaw_spinbox_, 1, 2);

  QGroupBox* setpoints_group_box = new QGroupBox(tr("Setpoints"));
  setpoints_group_box->setLayout(setpoints_layout);

  QGridLayout* topics_layout = new QGridLayout;
  topics_layout->addWidget(new QLabel("Pitch"), 0, 0);
  topics_layout->addWidget(pitch_topic_editor_, 0, 1);
  topics_layout->addWidget(new QLabel("Yaw"), 1, 0);
  topics_layout->addWidget(yaw_topic_editor_, 1, 1);

  QGroupBox* topics_group_box = new QGroupBox(tr("Topics"));
  topics_group_box->setLayout(topics_layout);

  QHBoxLayout* enabled_layout = new QHBoxLayout;
  QSizePolicy enabled_check_box_policy;
  enabled_check_box_policy.setHorizontalStretch(0);
  enabled_check_box_->setSizePolicy(enabled_check_box_policy);
  enabled_layout->addWidget(enabled_check_box_);
  enabled_layout->addWidget(new QLabel("Enabled"));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(setpoints_group_box);
  layout->addWidget(topics_group_box);
  layout->addLayout(enabled_layout);
  setLayout(layout);

  //===========================================================================
  // connections
  //===========================================================================

  // link setpoint sliders and spinboxes
  connect(pitch_slider_, SIGNAL(valueChanged(int)), pitch_spinbox_, SLOT(setValue(int)));
  connect(pitch_spinbox_, SIGNAL(valueChanged(int)), pitch_slider_, SLOT(setValue(int)));
  connect(yaw_slider_, SIGNAL(valueChanged(int)), yaw_spinbox_, SLOT(setValue(int)));
  connect(yaw_spinbox_, SIGNAL(valueChanged(int)), yaw_slider_, SLOT(setValue(int)));

  // link events to callbacks
  connect(pitch_slider_, SIGNAL(valueChanged(int)), this, SLOT(setPitch(int)));
  connect(yaw_slider_, SIGNAL(valueChanged(int)), this, SLOT(setYaw(int)));

  connect(pitch_topic_editor_, SIGNAL(editingFinished()), this, SLOT(updatePitchTopic()));
  connect(yaw_topic_editor_, SIGNAL(editingFinished()), this, SLOT(updateYawTopic()));

  connect(enabled_check_box_, SIGNAL(toggled(bool)), this, SLOT(setEnabled(bool)));

  connect(setpoint_timer, SIGNAL(timeout()), this, SLOT(sendSetpoints()));

  //===========================================================================
  // final setup
  //===========================================================================

  // initialize values
  pitch_slider_->setValue(0);
  yaw_slider_->setValue(0);

  pitch_topic_editor_->setText("/theta_r");
  yaw_topic_editor_->setText("/psi_r");

  // initialize publishers
  updatePitchTopic();
  updateYawTopic();
  setEnabled(enabled_check_box_->isChecked());

  // start the timer
  setpoint_timer->start(100);
}

void WhirlybirdPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString topic;
  if (config.mapGetString("PitchTopic", &topic))
  {
    pitch_topic_editor_->setText(topic);
    updatePitchTopic();
  }
  if (config.mapGetString("YawTopic", &topic))
  {
    yaw_topic_editor_->setText(topic);
    updateYawTopic();
  }

  bool enabled;
  if (config.mapGetBool("Enabled", &enabled))
  {
    enabled_check_box_->setChecked(enabled);
    setEnabled(enabled);
  }
}

void WhirlybirdPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("PitchTopic", pitch_topic_);
  config.mapSetValue("YawTopic", yaw_topic_);
  config.mapSetValue("Enabled", enabled_);
}

void WhirlybirdPanel::setPitch(int pitch_deg)
{
  pitch_setpoint_rad_ = pitch_deg * (M_PI / 180);
}

void WhirlybirdPanel::setYaw(int yaw_deg)
{
  yaw_setpoint_rad_ = yaw_deg * (M_PI / 180);
}

void WhirlybirdPanel::setPitchTopic(const QString& topic)
{
  if (topic != pitch_topic_)
  {
    pitch_topic_ = topic;
    Q_EMIT configChanged();
  }

  if (pitch_topic_.isEmpty() || !enabled_)
  {
    pitch_publisher_.shutdown();
  }
  else
  {
    pitch_publisher_ = nh_.advertise<std_msgs::Float32>(pitch_topic_.toStdString(), 1);
  }

  pitch_slider_->setEnabled(!pitch_topic_.isEmpty() && enabled_);
  pitch_spinbox_->setEnabled(pitch_slider_->isEnabled());
}

void WhirlybirdPanel::setYawTopic(const QString& topic)
{
  if (topic != yaw_topic_)
  {
    yaw_topic_ = topic;
    Q_EMIT configChanged();
  }

  if (yaw_topic_.isEmpty() || !enabled_)
  {
    yaw_publisher_.shutdown();
  }
  else
  {
    yaw_publisher_ = nh_.advertise<std_msgs::Float32>(yaw_topic_.toStdString(), 1);
  }

  yaw_slider_->setEnabled(!yaw_topic_.isEmpty() && enabled_);
  yaw_spinbox_->setEnabled(yaw_slider_->isEnabled());
}

void WhirlybirdPanel::setEnabled(bool enabled)
{
  if (enabled != enabled_)
  {
    Q_EMIT configChanged();

    enabled_ = enabled;
    updatePitchTopic();
    updateYawTopic();
  }
}

void WhirlybirdPanel::sendSetpoints()
{
  if (ros::ok())
  {
    if (pitch_publisher_)
    {
      std_msgs::Float32 pitch_msg;
      pitch_msg.data = pitch_setpoint_rad_;
      pitch_publisher_.publish(pitch_msg);
    }

    if (yaw_publisher_)
    {
      std_msgs::Float32 yaw_msg;
      yaw_msg.data = yaw_setpoint_rad_;
      yaw_publisher_.publish(yaw_msg);
    }
  }
}

void WhirlybirdPanel::updatePitchTopic()
{
  setPitchTopic(pitch_topic_editor_->text());
}

void WhirlybirdPanel::updateYawTopic()
{
  setYawTopic(yaw_topic_editor_->text());
}

} // namespace whirlybird_description

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whirlybird_description::WhirlybirdPanel, rviz::Panel)
