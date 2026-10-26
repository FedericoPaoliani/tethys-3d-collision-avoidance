// Carrello della telecamera per i video delle simulazioni (solo visualizzazione).
//
// Sposta il modello statico che porta la telecamera seguendo il drone <target> in x e y con
// un filtro del primo ordine (costante di tempo tau): posizione = drone + (offset_x, offset_y),
// quota z fissa, orientamento fisso. Il modello non ha geometria: non interagisce con la fisica
// né con i sonar. Quando il drone esce dalla vasca (arrivato al goal e spostato fuori
// dall'harness) il carrello resta fermo.
//
// Parametri SDF: <target>, <offset_x>, <offset_y>, <z>, <tau>, <limite_x>, <limite_y>.

#include <chrono>
#include <cmath>
#include <string>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>

namespace video
{
class TelecameraVideo : public ignition::gazebo::System,
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
    offset_x_ = _sdf->Get<double>("offset_x", offset_x_).first;
    offset_y_ = _sdf->Get<double>("offset_y", offset_y_).first;
    z_ = _sdf->Get<double>("z", z_).first;
    tau_ = _sdf->Get<double>("tau", tau_).first;
    limite_x_ = _sdf->Get<double>("limite_x", limite_x_).first;
    limite_y_ = _sdf->Get<double>("limite_y", limite_y_).first;
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

    const auto &d = drone_pose->Data().Pos();
    if (std::abs(d.X()) > limite_x_ || std::abs(d.Y()) > limite_y_)
      return;  // drone parcheggiato fuori dalla vasca: il carrello resta dov'è
    const double gx = d.X() + offset_x_;
    const double gy = d.Y() + offset_y_;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (!inizializzato_)
    {
      x_ = gx;
      y_ = gy;
      inizializzato_ = true;
    }
    else
    {
      const double a = 1.0 - std::exp(-dt / tau_);
      x_ += (gx - x_) * a;
      y_ += (gy - y_) * a;
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
  double offset_x_{-5.0};
  double offset_y_{-10.0};
  double z_{18.0};
  double tau_{1.5};
  double limite_x_{40.0};
  double limite_y_{70.0};
  double x_{0.0};
  double y_{0.0};
  bool inizializzato_{false};
};
}  // namespace video

IGNITION_ADD_PLUGIN(video::TelecameraVideo, ignition::gazebo::System,
                    video::TelecameraVideo::ISystemConfigure,
                    video::TelecameraVideo::ISystemPreUpdate)
