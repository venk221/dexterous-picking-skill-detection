#include <ignition/math/Pose3.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

namespace gazebo {
  class BBPlugin : public WorldPlugin {
  public:
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
      // Save the list of models
      models = _parent->GetModels();
      modelFilter = ".";

      // Load custom supplied regex if available
      if(!_sdf->HasElement("regx"))
	gzwarn << "No regx specified, using all\n";
      else
	modelFilter = _sdf->getElement("regx")->Get<std::string>();

      this->updateConn = event::Events::ConnectWorldUpdateBegin(std::bind(&BBPlugin::OnUpdate, this));
    }

    void OnUpdate() {
      ignition::math::Box bb;
      for(ModelPtr mp : this.models) {
	bb = mp->BoundingBox();
      }
    }
  private:
    std::vector<ModelPtr> models;
    std::regex modelFilter;
    event::ConnectionPtr updateConn;
  };
}
