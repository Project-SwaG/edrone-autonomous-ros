#include <algorithm>
#include <assert.h>
#include "vitarana_drone/gazebo_edrone_propulsion.h"

namespace gazebo
{

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelJointControler);

  // Constructor
  ModelJointControler::ModelJointControler()
  {
    this->force_msg_.prop1=0;
    this->force_msg_.prop2=0;
    this->force_msg_.prop3=0;
    this->force_msg_.prop4=0;

    this->temp_msg_.prop1=0;
    this->temp_msg_.prop2=0;
    this->temp_msg_.prop3=0;
    this->temp_msg_.prop4=0;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  ModelJointControler::~ModelJointControler()
  {
    // event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

    // Custom Callback Queue
    this->queue_.clear();
    this->queue_.disable();
    this->rosnode_->shutdown();
    this->callback_queue_thread_.join();

    delete this->rosnode_;
  }


  // Load the controller
  void ModelJointControler::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model = _parent;

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelJointControler::OnUpdate, this));
    
    this->old_secs =ros::Time::now().toSec();

  //************************************************************************************************************************************************************************
    if (_sdf->HasElement("prop_kp"))
        this->prop_kp = _sdf->Get<double>("prop_kp");
    if (_sdf->HasElement("prop_ki"))
        this->prop_ki = _sdf->Get<double>("prop_ki");
    if (_sdf->HasElement("prop_kd"))
        this->prop_kd = _sdf->Get<double>("prop_kd");
    if (_sdf->HasElement("robotNamespace"))
        this->robot_namespace_ = _sdf->Get<std::string>("robotNamespace");
    if (_sdf->HasElement("activate_pid_control"))
        this->activate_pid_control = (_sdf->Get<std::string>("activate_pid_control") == "yes");
      
    if (!_sdf->HasElement("bodyName_1"))
    {
      ROS_FATAL("force plugin missing <bodyName_1>, cannot proceed");
      return;
    }
    else
      this->link1_name_ = _sdf->GetElement("bodyName_1")->Get<std::string>();

    this->link1_ = model->GetLink(this->link1_name_);
    if (!this->link1_)
    {
      ROS_FATAL("gazebo_ros_force plugin error: link named: %s does not exist\n",this->link1_name_.c_str());
      return;
    }
  //******************
    if (!_sdf->HasElement("bodyName_2"))
    {
      ROS_FATAL("force plugin missing <bodyName_2>, cannot proceed");
      return;
    }
    else
      this->link2_name_ = _sdf->GetElement("bodyName_2")->Get<std::string>();

    this->link2_ = model->GetLink(this->link2_name_);
    if (!this->link2_)
    {
      ROS_FATAL("gazebo_ros_force plugin error: link named: %s does not exist\n",this->link2_name_.c_str());
      return;
    }
  //*********************
    if (!_sdf->HasElement("bodyName_3"))
    {
      ROS_FATAL("force plugin missing <bodyName_3>, cannot proceed");
      return;
    }
    else
      this->link3_name_ = _sdf->GetElement("bodyName_3")->Get<std::string>();

    this->link3_ = model->GetLink(this->link3_name_);
    if (!this->link3_)
    {
      ROS_FATAL("gazebo_ros_force plugin error: link named: %s does not exist\n",this->link3_name_.c_str());
      return;
    }
  //**********************
    if (!_sdf->HasElement("bodyName_4"))
    {
      ROS_FATAL("force plugin missing <bodyName_4>, cannot proceed");
      return;
    }
    else
      this->link4_name_ = _sdf->GetElement("bodyName_4")->Get<std::string>();

    this->link4_ = model->GetLink(this->link4_name_);
    if (!this->link4_)
    {
      ROS_FATAL("gazebo_ros_force plugin error: link named: %s does not exist\n",this->link4_name_.c_str());
      return;
    }

  //*************************************************************************************************

    if (!_sdf->HasElement("topicName"))
    {
      ROS_FATAL("force plugin missing <topicName>, cannot proceed");
      return;
    }
    else
      this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  //*****************************************************************************************************************************************************************************  


    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    
    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    // Create subscriber for prop speed
    ros::SubscribeOptions so = ros::SubscribeOptions::create<vitarana_drone::prop_speed>(
      this->topic_name_,1,
      boost::bind( &ModelJointControler::OnRosMsg_prop_speed,this,_1),
      ros::VoidPtr(), &this->queue_);
    this->sub_ = this->rosnode_->subscribe(so);
    
    // Spin up the queue helper thread.
    this->callback_queue_thread_ = boost::thread( boost::bind( &ModelJointControler::QueueThread,this ) );

    ROS_INFO("Loaded Plugin with parent...%s", this->model->GetName().c_str());
    

    if(this->activate_pid_control)
    {
      // Activated PID Speed Control
      const auto &jointController = this->model->GetJointController();
      jointController->Reset();

      jointController->AddJoint(model->GetJoint("prop1_joint"));
      jointController->AddJoint(model->GetJoint("prop2_joint"));
      jointController->AddJoint(model->GetJoint("prop3_joint"));
      jointController->AddJoint(model->GetJoint("prop4_joint"));

      this->prop1_name = model->GetJoint("prop1_joint")->GetScopedName();
      this->prop2_name = model->GetJoint("prop2_joint")->GetScopedName();
      this->prop3_name = model->GetJoint("prop3_joint")->GetScopedName();
      this->prop4_name = model->GetJoint("prop4_joint")->GetScopedName();
      
      jointController->SetVelocityPID(this->prop1_name, common::PID(this->prop_kp, this->prop_ki, this->prop_kd));
      jointController->SetVelocityPID(this->prop2_name, common::PID(this->prop_kp, this->prop_ki, this->prop_kd));
      jointController->SetVelocityPID(this->prop3_name, common::PID(this->prop_kp, this->prop_ki, this->prop_kd));
      jointController->SetVelocityPID(this->prop4_name, common::PID(this->prop_kp, this->prop_ki, this->prop_kd));
      
      jointController->SetVelocityTarget(this->prop1_name, 0.0);
      jointController->SetVelocityTarget(this->prop2_name, 0.0);
      jointController->SetVelocityTarget(this->prop3_name, 0.0);
      jointController->SetVelocityTarget(this->prop4_name, 0.0);
    }
    ROS_INFO("Vitarana Drone version 0.2.1");
    
  }



  // Called by the world update start event
  void ModelJointControler::OnUpdate()
  {
    double new_secs =ros::Time::now().toSec();
    double delta = new_secs - this->old_secs;
    
    double max_delta = 0.0;
    
    if (this->freq_update != 0.0)
    {
      max_delta = 1.0 / this->freq_update;
    }
    
    if (delta > max_delta && delta != 0.0)
    {
      this->old_secs = new_secs;

    if(this->activate_pid_control)
      {
        ROS_DEBUG("Update propeller Speed PID...");
        const auto &jointController = this->model->GetJointController();
        jointController->SetVelocityTarget(this->prop1_name, this->prop1_speed_magn);
        jointController->SetVelocityTarget(this->prop2_name, this->prop2_speed_magn);
        jointController->SetVelocityTarget(this->prop3_name, this->prop3_speed_magn);
        jointController->SetVelocityTarget(this->prop4_name, this->prop4_speed_magn);
        jointController->Update();
      }
    else
      {
        // Apply a small linear velocity to the model.
        ROS_DEBUG("Update propeller Speed BASIC...");
        this->model->GetJoint("prop1_joint")->SetVelocity(0, this->prop1_speed_magn);
        this->model->GetJoint("prop2_joint")->SetVelocity(0, this->prop2_speed_magn);
        this->model->GetJoint("prop3_joint")->SetVelocity(0, this->prop3_speed_magn);
        this->model->GetJoint("prop4_joint")->SetVelocity(0, this->prop4_speed_magn);

      }

    }

    this->lock_.lock();
    // math::Vector3d force(0,0,this->force_msg_.prop1);
    this->link1_->AddRelativeForce(ignition::math::Vector3d(0,0,this->force_msg_.prop1));
    this->link2_->AddRelativeForce(ignition::math::Vector3d(0,0,this->force_msg_.prop2));
    this->link3_->AddRelativeForce(ignition::math::Vector3d(0,0,this->force_msg_.prop3));
    this->link4_->AddRelativeForce(ignition::math::Vector3d(0,0,this->force_msg_.prop4));
    this->lock_.unlock();
  }
          
  void ModelJointControler::OnRosMsg_prop_speed(const vitarana_drone::prop_speedConstPtr &_msg)
  {

    this->temp_msg_.prop1=_msg->prop1;
    this->temp_msg_.prop2=_msg->prop2;
    this->temp_msg_.prop3=_msg->prop3;
    this->temp_msg_.prop4=_msg->prop4;

// get in range from 0 to 1024
    if (this->temp_msg_.prop1>=1024.0)
      this->temp_msg_.prop1=1024;
    if (this->temp_msg_.prop2>=1024.0)
      this->temp_msg_.prop2=1024;
    if (this->temp_msg_.prop3>=1024.0)
      this->temp_msg_.prop3=1024;
    if (this->temp_msg_.prop4>=1024.0)
      this->temp_msg_.prop4=1024;

    if (this->temp_msg_.prop1<0)
      this->temp_msg_.prop1=0;
    if (this->temp_msg_.prop2<0)
      this->temp_msg_.prop2=0;
    if (this->temp_msg_.prop3<0)
      this->temp_msg_.prop3=0;
    if (this->temp_msg_.prop4<0)
      this->temp_msg_.prop4=0;
    
    
    this->prop1_speed_magn = this->temp_msg_.prop1/20;
    this->prop2_speed_magn = this->temp_msg_.prop2/20;
    this->prop3_speed_magn = this->temp_msg_.prop3/20;
    this->prop4_speed_magn = this->temp_msg_.prop4/20;
    this->force_msg_.prop1 = this->temp_msg_.prop1/146.28;
    this->force_msg_.prop2 = this->temp_msg_.prop2/146.28;
    this->force_msg_.prop3 = this->temp_msg_.prop3/146.28;
    this->force_msg_.prop4 = this->temp_msg_.prop4/146.28;


  }

  // brief ROS helper function that processes messages
  void ModelJointControler::QueueThread()
  {
    static const double timeout = 0.01;

    while (this->rosnode_->ok())
    {
      this->queue_.callAvailable(ros::WallDuration(timeout));
    }
  }
}
