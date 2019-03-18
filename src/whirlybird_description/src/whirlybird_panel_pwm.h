/**
 * \file whirlybird_panel.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 *
 * Rviz panel plugin for setting whirlybird setpoints
 */

#ifndef WHIRLYBIRD_PANEL_PWM_H
#define WHIRLYBIRD_PANEL_PWM_H

#ifndef Q_MOC_RUN
  #include <ros/ros.h>
  #include <rviz/panel.h>
#endif

class QLineEdit;
class QSlider;
class QSpinBox;
class QCheckBox;
class QString;

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

namespace whirlybird_description
{

class WhirlybirdPanelPWM : public rviz::Panel
{
Q_OBJECT
public:
  WhirlybirdPanelPWM(QWidget* parent);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:
  void setPWM(int pwm);
  void setPWMTopic(const QString& topic);
  void setEnabled(bool enabled);

protected Q_SLOTS:
  void sendSetpoints();
  void updatePWMTopic();

protected:
  QLineEdit* pwm_topic_editor_;

  QSlider* pwm_slider_;

  QSpinBox* pwm_spinbox_;

  QString pwm_topic_;

  QCheckBox* enabled_check_box_;

  ros::Publisher pwm_publisher_;

  ros::NodeHandle nh_;

  bool enabled_;

  float pwm_setpoint_;

  int pwm_max_;
  int pwm_min_;
};

} // namespace whirlybird_description

#endif // WHIRLYBIRD_PANEL_PWM_H
