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
#include <whirlybird_msgs/Command.h>

#include "whirlybird_panel_pwm.h"

namespace whirlybird_description
{

WhirlybirdPanelPWM::WhirlybirdPanelPWM(QWidget* parent = 0) : rviz::Panel(parent),
  pwm_setpoint_(0.0f),
  pwm_max_(80),
  pwm_min_(0)
{
  //===========================================================================
  // controls
  //===========================================================================

  // setpoints
  pwm_slider_ = new QSlider(Qt::Horizontal);
  pwm_slider_->setRange(pwm_min_, pwm_max_);
  pwm_slider_->setTickPosition(QSlider::TicksBelow);
  pwm_slider_->setTickInterval(10);

  pwm_spinbox_ = new QSpinBox;
  pwm_spinbox_->setRange(pwm_min_, pwm_max_);

  // topics
  pwm_topic_editor_ = new QLineEdit;

  // enabled
  enabled_check_box_ = new QCheckBox;

  // timer
  QTimer* setpoint_timer = new QTimer(this);

  //===========================================================================
  // layout
  //===========================================================================

  QGridLayout* setpoints_layout = new QGridLayout;
  setpoints_layout->addWidget(new QLabel("PWM"), 0, 0);
  setpoints_layout->addWidget(pwm_slider_, 0, 1);
  setpoints_layout->addWidget(pwm_spinbox_, 0, 2);

  QGroupBox* setpoints_group_box = new QGroupBox(tr("Setpoints"));
  setpoints_group_box->setLayout(setpoints_layout);

  QGridLayout* topics_layout = new QGridLayout;
  topics_layout->addWidget(new QLabel("PWM"), 0, 0);
  topics_layout->addWidget(pwm_topic_editor_, 0, 1);

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
  connect(pwm_slider_, SIGNAL(valueChanged(int)), pwm_spinbox_, SLOT(setValue(int)));
  connect(pwm_spinbox_, SIGNAL(valueChanged(int)), pwm_slider_, SLOT(setValue(int)));

  // link events to callbacks
  connect(pwm_slider_, SIGNAL(valueChanged(int)), this, SLOT(setPWM(int)));

  connect(pwm_topic_editor_, SIGNAL(editingFinished()), this, SLOT(updatePWMTopic()));

  connect(enabled_check_box_, SIGNAL(toggled(bool)), this, SLOT(setEnabled(bool)));

  connect(setpoint_timer, SIGNAL(timeout()), this, SLOT(sendSetpoints()));

  //===========================================================================
  // final setup
  //===========================================================================

  // initialize values
  pwm_slider_->setValue(0);

  pwm_topic_editor_->setText("/command");

  // initialize publishers
  updatePWMTopic();
  setEnabled(enabled_check_box_->isChecked());

  // start the timer
  setpoint_timer->start(100);
}

void WhirlybirdPanelPWM::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString topic;
  if (config.mapGetString("PWMTopic", &topic))
  {
    pwm_topic_editor_->setText(topic);
    updatePWMTopic();
  }

  bool enabled;
  if (config.mapGetBool("Enabled", &enabled))
  {
    enabled_check_box_->setChecked(enabled);
    setEnabled(enabled);
  }
}

void WhirlybirdPanelPWM::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("PWMTopic", pwm_topic_);
  config.mapSetValue("Enabled", enabled_);
}

void WhirlybirdPanelPWM::setPWM(int pwm)
{
  pwm_setpoint_ = 0.01*pwm;
}

void WhirlybirdPanelPWM::setPWMTopic(const QString& topic)
{
  if (topic != pwm_topic_)
  {
    pwm_topic_ = topic;
    Q_EMIT configChanged();
  }

  if (pwm_topic_.isEmpty() || !enabled_)
  {
    pwm_publisher_.shutdown();
  }
  else
  {
    pwm_publisher_ = nh_.advertise<whirlybird_msgs::Command>(pwm_topic_.toStdString(), 1);
  }

  pwm_slider_->setEnabled(!pwm_topic_.isEmpty() && enabled_);
  pwm_spinbox_->setEnabled(pwm_slider_->isEnabled());
}

void WhirlybirdPanelPWM::setEnabled(bool enabled)
{
  if (enabled != enabled_)
  {
    Q_EMIT configChanged();

    enabled_ = enabled;
    updatePWMTopic();
  }
}

void WhirlybirdPanelPWM::sendSetpoints()
{
  if (ros::ok())
  {
    if (pwm_publisher_)
    {
      whirlybird_msgs::Command pwm_msg;
      pwm_msg.left_motor = pwm_setpoint_;
      pwm_msg.right_motor = pwm_setpoint_;
      pwm_publisher_.publish(pwm_msg);
    }
  }
}

void WhirlybirdPanelPWM::updatePWMTopic()
{
  setPWMTopic(pwm_topic_editor_->text());
}

} // namespace whirlybird_description

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whirlybird_description::WhirlybirdPanelPWM, rviz::Panel)
