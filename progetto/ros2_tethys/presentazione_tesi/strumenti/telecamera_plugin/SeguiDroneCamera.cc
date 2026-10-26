// Carrello della telecamera per il video dimostrativo (solo mondo demo).
//
// Sposta il modello statico che porta la telecamera lungo l'asse Y del corridoio,
// dietro al drone, con un filtro del primo ordine (costante di tempo tau). x, z e
// l'orientamento restano fissi: la telecamera non ruota con il drone, quindi la
// deviazione laterale si vede rispetto al pilastro e al centro del corridoio.
// Il modello non ha geometria: non interagisce con la fisica né con i sonar.
//
// Parametri SDF: <target> (nome del modello da seguire), <offset_y>, <x>, <z>, <tau>.

#include <cmath>
#include <string>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>

namespace demo
{
class SeguiDroneCamera : public ignition::gazebo::System,
                         public ignition::gazebo::ISystemConfigure,
                         public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &,
                 ignition::gazebo::EventManager &) override
  {
    rig_ = _entity;
    target_ = _sdf->Get<std::string>("target", target_).first;
    offset_y_ = _sdf->Get<double>("offset_y", offset_y_).first;
    x_ = _sdf->Get<double>("x", x_).first;
    z_ = _sdf->Get<double>("z", z_).first;
    tau_ = _sdf->Get<double>("tau", tau_).first;
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    if (drone_ == ignition::gazebo::kNullEntity)
    {
      drone_ = _ecm.EntityByComponents(ignition::gazebo::components::Model(),
                                       ignition::gazebo::components::Name(target_));
      if (drone_ == ignition::gazebo::kNullEntity)
        return;
    }
    auto *drone_pose = _ecm.Component<ignition::gazebo::components::Pose>(drone_);
    auto *rig_pose = _ecm.Component<ignition::gazebo::components::Pose>(rig_);
    if (!drone_pose || !rig_pose)
      return;

    const double goal_y = drone_pose->Data().Pos().Y() + offset_y_;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (!initialised_)
    {
      y_ = goal_y;
      initialised_ = true;
    }
    else
    {
      y_ += (goal_y - y_) * (1.0 - std::exp(-dt / tau_));
    }

    ignition::math::Pose3d p = rig_pose->Data();
    p.Pos().Set(x_, y_, z_);
    rig_pose->Data() = p;
    _ecm.SetChanged(rig_, ignition::gazebo::components::Pose::typeId,
                    ignition::gazebo::ComponentState::OneTimeChange);
  }

private:
  ignition::gazebo::Entity rig_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity drone_{ignition::gazebo::kNullEntity};
  std::string target_{"tethys"};
  double offset_y_{-9.0};
  double x_{0.0};
  double z_{13.0};
  double tau_{1.0};
  double y_{0.0};
  bool initialised_{false};
};
}  // namespace demo

IGNITION_ADD_PLUGIN(demo::SeguiDroneCamera, ignition::gazebo::System,
                    demo::SeguiDroneCamera::ISystemConfigure,
                    demo::SeguiDroneCamera::ISystemPreUpdate)
