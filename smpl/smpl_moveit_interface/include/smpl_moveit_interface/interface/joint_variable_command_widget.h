#ifndef sbpl_interface_joint_variable_command_widget_h
#define sbpl_interface_joint_variable_command_widget_h

// standard includes
#include <map>
#include <vector>

#include <QWidget>

#ifndef Q_MOC_RUN
#include <moveit/robot_model/robot_model.h>
#endif

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QString;

namespace sbpl_interface {

class RobotCommandModel;

/// A widget providing spinbox controls for joint variables in a RobotModel.
/// Upon construction, this widget automatically subscribes to the robotLoaded()
/// and robotStateChanged() signals from its associated RobotCommandModel.
///
/// This widget maintains an active joint group, which is the joint group for
/// which spinbox variable controls are provided. An active joint group may be
/// set before a RobotModel is loaded, but after a RobotModel is loaded, the
/// active joint group must be one of the joint groups within the RobotModel.
class JointVariableCommandWidget : public QWidget
{
    Q_OBJECT

public:

    JointVariableCommandWidget(RobotCommandModel* model, QWidget* parent = 0);
    ~JointVariableCommandWidget();

    auto activeJointGroup() const -> const std::string&
    { return m_active_joint_group; }

public Q_SLOTS:

    void setActiveJointGroup(const std::string& group_name);
    void setActiveNamedState(const std::string& state_name);

Q_SIGNALS:

    // emitted whenever the active joint group changes
    void updateActiveJointGroup(const std::string& group_name);

private:

    RobotCommandModel* m_model;
    std::string m_active_joint_group;

    QComboBox* m_joint_groups_combo_box;
    QComboBox* m_joint_group_states_combo_box;

    std::vector<QDoubleSpinBox*> m_spinboxes;
    std::vector<QLabel*> m_labels;

    // mapping from each qdoublespinbox to the indices of the joint variables it
    // controls...really this is a huge over-generalization to allow a set of
    // (r, p, y) spinboxes to control a set of (qw, qx, qy, qz) joint variables.
    // In those places, the indices are expected refer to joint variables in the
    // order (qw, qx, qy, qz).
    std::map<QDoubleSpinBox*, std::vector<int>> m_spinbox_to_vind;

    // mapping from variable index to the spinboxes that affects its value. As
    // above, this is to support RPY spinboxes. and in those cases, the
    // expected order of these spinboxes is (r, p, y)
    std::vector<std::vector<QDoubleSpinBox*>> m_vind_to_spinbox;

    bool m_ignore_sync;

    void buildRobotControls();

    void updateJointGroupControls();

private Q_SLOTS:

    void updateRobotModel();
    void updateRobotState();

    // called when the item in the combo box changes
    void setJointGroup(const QString& group_name);
    void setNamedState(const QString& state_name);

    void setJointVariableFromSpinBox(double value);
};

} // namespace sbpl_interface

#endif
