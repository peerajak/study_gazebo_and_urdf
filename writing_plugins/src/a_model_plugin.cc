#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {
class MyModelPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the model
    this->model = _parent;

    this->iterations = 10 * 1000;
    if (_sdf->HasElement("iterations")) {
      this->iterations = _sdf->Get<int>("iterations");
    }
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MyModelPlugin::OnUpdate, this));
  }

  // Called by the world update start event
public:
  void OnUpdate() {
    // Apply a small linear velocity to the model.

    if (this->counter < this->iterations) {
      this->model->SetLinearVel(ignition::math::Vector3d(1.0, 0, 0));
      this->model->SetLinearVel(ignition::math::Vector3d(-1.0, 0, 0));
      //   this->model->SetLinearAccel(ignition::math::Vector3d(0, 0, 0));
    }

    this->counter++;
  }

  // Pointer to the model
private:
  physics::ModelPtr model;

private:
  int counter;
  int iterations;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MyModelPlugin)
} // namespace gazebo