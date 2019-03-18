/**
 * \file whirlybird_panel.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 *
 * Rviz panel plugin for setting whirlybird setpoints
 */

#ifndef WHIRLYBIRD_PANEL_H
#define WHIRLYBIRD_PANEL_H

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

class WhirlybirdPanel : public rviz::Panel
{
Q_OBJECT
public:
  WhirlybirdPanel(QWidget* parent);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:
  void setPitch(int pitch_deg);
  void setYaw(int yaw_deg);
  void setPitchTopic(const QString& topic);
  void setYawTopic(const QString& topic);
  void setEnabled(bool enabled);

protected Q_SLOTS:
  void sendSetpoints();
  void updatePitchTopic();
  void updateYawTopic();

protected:
  QLineEdit* pitch_topic_editor_;
  QLineEdit* yaw_topic_editor_;

  QSlider* pitch_slider_;
  QSlider* yaw_slider_;

  QSpinBox* pitch_spinbox_;
  QSpinBox* yaw_spinbox_;

  QString pitch_topic_;
  QString yaw_topic_;

  QCheckBox* enabled_check_box_;

  ros::Publisher pitch_publisher_;
  ros::Publisher yaw_publisher_;

  ros::NodeHandle nh_;

  bool enabled_;

  float pitch_setpoint_rad_;
  float yaw_setpoint_rad_;

  int pitch_max_deg_;
  int yaw_max_deg_;
};

} // namespace whirlybird_description

#endif // WHIRLYBIRD_PANEL_H
