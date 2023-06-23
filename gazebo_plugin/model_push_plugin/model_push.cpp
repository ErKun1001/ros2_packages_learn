#include <functional>
#include <chrono>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
    class ModelPush : public ModelPlugin
    {
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ModelPush::OnUpdate, this));
        }

        // Called by the world update start event
        public: void OnUpdate()
        {
            auto now = std::chrono::system_clock::now();
            std::cout<<"now time point: "<< std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()<<std::endl;
            time_t tt = std::chrono::system_clock::to_time_t(now);
            std::cout<<"now is "<<ctime(&tt)<<std::endl;

            // Apply a small linear velocity to the model.
            this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
        }

        // Pointer to the model
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}