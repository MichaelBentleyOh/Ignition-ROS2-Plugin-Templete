/// @brief Plugin Template for AstroROS in source file

// Original license is Apache-2.0 license. License overlap has to be considered.
// MIT License

// Copyright (c) 2025 Unmanned Systems Control Lab@KAU

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// C++
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>

// Include Ignition Gazebo headers
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>

#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/AngularAcceleration.hh>

#include <ignition/plugin/Register.hh>

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>

namespace ign_plugin // Name your plugin namespace
{

class IgnitionPlugin // Name your plugin class
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate // these inheritances are fixed.
{
public:

  /// \brief Constructor
  IgnitionPlugin(){

    // We assume ROS has already been initialized externally, but if not:
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);  // or handle arguments properly
    }

  };

  /// \brief Destructor
  ~IgnitionPlugin() override
  {
    // Shutdown ROS and join spin thread
    rclcpp::shutdown();
    if (this->ros_spin_thread_.joinable())
    {
      this->ros_spin_thread_.join();
    }

  }

  /// \brief Configure: Called once at startup
  /*
  _entity : objects in ignition.(world,gui,robot etc.) You will add this plugin into xacro file.
            So you could think this entity is robot "model" you want.
  _sdf    : sdf/xacro/urdf file. It means an object of sdf you wrote; the file that this plugin
            added.
  _ecm    : Entity manager for ignition entities. every entities are monitored by it.
  &       : If there is a factor in original function but do not use that factor, then "&" could be
            used without warning. (TBR)
  */
  void Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &) override
  {
    // For demonstration, get link name from SDF parameter (or you could hard-code)
    if (_sdf->HasElement("model_name")) {
      this->model_name_ = _sdf->Get<std::string>("model_name");
    } else {
      std::cout<<"[ERROR] Missing <model_name> parameter in SDF. Check your SDF/Xacro/URDF file."<<std::endl;
      return;
    }
    // Create an rclcpp::Node and an other data handling method in this part
    this->node_ = std::make_shared<rclcpp::Node>(this->model_name_+"_plugin_node");

    RCLCPP_INFO(this->node_->get_logger(),"[%s_plugin] Find model name : [%s]",
                this->model_name_.c_str(),
                this->model_name_.c_str());

    this->model_pose_pub_ = this->node_->create_publisher<std_msgs::msg::Float32MultiArray>(
                            this->model_name_+"_data", 10);
    this->model_pose_timer_ = this->node_->create_wall_timer(
    std::chrono::milliseconds(100),  // 10 Hz timer
    std::bind(&IgnitionPlugin::ModelPoseCallback, this));

    // Make ros thread so that plugin node can operate indepenetly

    this->ros_spin_thread_ = std::thread([this]() {
      rclcpp::executors::MultiThreadedExecutor executor;
      executor.add_node(this->node_);
      RCLCPP_INFO(this->node_->get_logger(),
      "[%s_plugin] ROS 2 MultiThreadedExecutor started.",
      this->model_name_.c_str());
      executor.spin();  // Use MultiThreadedExecutor for safer threading
    });

    if (_sdf->HasElement("link_name")) {
      this->link_name_ = _sdf->Get<std::string>("link_name");
    } else {
      RCLCPP_ERROR(this->node_->get_logger(),
      "[%s_plugin] Missing <link_name> parameter in SDF.",
      this->model_name_.c_str());
      return;
    }
    RCLCPP_INFO(this->node_->get_logger(),"[%s_plugin] Find link name : [%s]",
                this->model_name_.c_str(),
                this->link_name_.c_str());

    // Attempt to locate the model entity in ECM
    auto model = ignition::gazebo::Model(_entity);
    if (!model.Valid(_ecm)) {
      RCLCPP_ERROR(this->node_->get_logger(),
      "[%s_plugin] This plugin must be attached to a valid model.",
      this->model_name_.c_str());
      return;
    }
    this->model_entity_ = model.Entity();
    auto model_entity_name = _ecm.Component<ignition::gazebo::components::Name>(this->model_entity_);

    if (model_entity_name->Data() != this->model_name_){
      RCLCPP_ERROR(this->node_->get_logger(),
                  "[%s_plugin] Unmatched entity name: [%s] [%s].",
                  this->model_name_.c_str(),
                  model_entity_name->Data().c_str(),
                  this->model_name_.c_str());
      return;
    }
    RCLCPP_INFO(this->node_->get_logger(),"[%s_plugin] Model name matched : [%s] [%s]",
                this->model_name_.c_str(),
                model_entity_name->Data().c_str(),
                this->model_name_.c_str());

    // Below is another simple way to get link entity by hard coding
    // this->linkEntity_ = _ecm.Component<ignition::gazebo::components::Link>(this->linkName_);
    auto link_entities = _ecm.ChildrenByComponents(model.Entity(), ignition::gazebo::components::Link());
    for (auto &link_entity : link_entities) {
      auto link_comp_name = _ecm.Component<ignition::gazebo::components::Name>(link_entity);
      if (link_comp_name && link_comp_name->Data() == this->link_name_) {
        this->link_entity_ = link_entity;
        this->link_ = ignition::gazebo::Link(link_entity);
        RCLCPP_INFO(this->node_->get_logger(), "["++"_plugin] Found link [%s] (Entity=%lu).",
                    this->model_name_.c_str(),
                    link_comp_name->Data().c_str(),
                    static_cast<unsigned long>(link_entity));
      }
    }

    RCLCPP_INFO(this->node_->get_logger(),
                "[%s_plugin] Configure() complete for IgnitionPlugin.",
                this->model_name_.c_str());
  }

  /// \brief PreUpdate: Called every simulation iteration before physics update
  /*
  _info : update data in ignition simulation; puasing, dt, simulation time etc.
  _ecm : entity component manager
  */
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused) {
    return;  // Do nothing if simulation is paused
    }

    if (this->link_entity_ == ignition::gazebo::kNullEntity) {
    RCLCPP_WARN(this->node_->get_logger(), "Link entity not found, skipping update.");
    return;
    }
    ignition::math::Vector3d force(100, 0, 0); // Apply an 100N force along the X-axis
    ignition::math::Vector3d wrench(100, 0, 0);// Apply an 100Nm torque along the X-axis
    this->link_.AddWorldWrench(_ecm, force, wrench);
  }
  //===============================================
  /// \brief PostUpdate: Called after physics update, so we can read final states
  /*
  _info : update data in ignition simulation; puasing, dt, simulation time etc.
  _ecm : entity component manager
  */
  void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                  const ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused) {
      return;
    }

    this->model_data_.clear();
    auto model_pose = _ecm.Component<ignition::gazebo::components::Pose>(this->model_entity_);

    if (!model_pose) {
      RCLCPP_WARN(this->node_->get_logger(),
      "[%s_plugin] Model pose component not found.",
      this->model_name_.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(this->data_mutex_);
    const auto &model_pose_data = model_pose->Data();

    // model world position (ECI frame)
    this->model_data_.push_back(model_pose_data.Pos().X());
    this->model_data_.push_back(model_pose_data.Pos().Y());
    this->model_data_.push_back(model_pose_data.Pos().Z());

    // model world orientation (ECI frame)
    this->model_data_.push_back(model_pose_data.Rot().W());
    this->model_data_.push_back(model_pose_data.Rot().X());
    this->model_data_.push_back(model_pose_data.Rot().Y());
    this->model_data_.push_back(model_pose_data.Rot().Z());
  }

private:

  std::mutex data_mutex_;  // mutex for avoiding race condition
  //===============================================
  // ROS 2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr model_pose_pub_;
  rclcpp::TimerBase::SharedPtr model_pose_timer_;
  std::thread ros_spin_thread_;

  //===============================================
  // Callback function
  void ModelPoseCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "model pose information";
    msg.layout.dim[0].size = this->model_data_.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.data_offset = 0;
    msg.data = this->model_data_;

    this->model_pose_pub_->publish(msg);
  }
  //===============================================
  // Link entity to which we apply a wrench
  std::string link_name_;
  ignition::gazebo::Entity link_entity_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Link link_;
  //===============================================
  // model entity and data
  std::string model_name_;
  ignition::gazebo::Entity model_entity_{ignition::gazebo::kNullEntity};
  std::vector<float> model_data_;
}; // class IgnitionPlugin

}  // namespace ign_plugin

// Register the plugin with Ignition
IGNITION_ADD_PLUGIN(
  ign_plugin::IgnitionPlugin, // this is the registration name for this plugin
  ignition::gazebo::System,
  ign_plugin::IgnitionPlugin::ISystemConfigure,
  ign_plugin::IgnitionPlugin::ISystemPreUpdate,
  ign_plugin::IgnitionPlugin::ISystemPostUpdate
)

// This is example of how to attach plugin in SDF/Xacro/URDF file.
// <gazebo>
//     <plugin filename="libign_plugin.so" name="ign_plugin::IgnitionPlugin">
//       <model_name>satellite</model_name>
//     </plugin>
// </gazebo>
