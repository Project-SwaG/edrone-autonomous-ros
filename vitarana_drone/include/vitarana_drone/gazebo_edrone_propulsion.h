#ifndef GAZEBO_ROS_FORCE_HH
#define GAZEBO_ROS_FORCE_HH

#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include "vitarana_drone/prop_speed.h"

#include <functional>
#include <thread>
#include <map>
#include <sdf/sdf.hh>


namespace gazebo
{

/*
  This is a Plugin that is used with vitarana_drone

  Example Usage:
      <gazebo>
        <plugin filename="libgazebo_ros_force.so" name="gazebo_ros_force">
          <bodyName>box_body</bodyName>
          <topicName>box_force</topicName>
        </plugin>
      </gazebo>
*/

class ModelJointControler : public ModelPlugin
{
  /// \brief Constructor
  public: ModelJointControler();

  /// \brief Destructor
  public: virtual ~ModelJointControler();

  // Documentation inherited
  protected: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief The custom callback queue thread function.
  private: void QueueThread();

  private: void OnRosMsg_prop_speed(const vitarana_drone::prop_speedConstPtr &_msg);
 
  private: void OnUpdate();


  /// \brief A pointer to the gazebo world.
  private: physics::WorldPtr world_;

  /// \brief A pointer to the Link, where force is applied
  private: physics::LinkPtr link1_;
  private: physics::LinkPtr link2_;
  private: physics::LinkPtr link3_;	
  private: physics::LinkPtr link4_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber sub_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock_;

  /// \brief ROS Wrench topic name inputs
  private: std::string topic_name_;
  /// \brief The Link this plugin is attached to, and will exert forces on.
  private: std::string link1_name_;
  private: std::string link2_name_;
  private: std::string link3_name_;
  private: std::string link4_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callback_queue_thread_;


  /// \brief Container for the wrench force that this plugin exerts on the body.
  private: vitarana_drone::prop_speed force_msg_;
  private: vitarana_drone::prop_speed temp_msg_;


  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
  // Time Memory
    double old_secs;
    
    // Frequency of earthquake
    double freq_update = 10.0;

    double prop1_speed_magn = 0.0;
    // Magnitude of the Oscilations
    double prop2_speed_magn = 0.0;

    double prop3_speed_magn = 0.0;

    double prop4_speed_magn = 0.0;

    std::string prop1_name;
    std::string prop2_name;
    std::string prop3_name;
    std::string prop4_name;
    
    double prop_kp = 0.1;
    double prop_ki = 0.0;
    double prop_kd = 0.0;
    bool activate_pid_control;




  // Pointer to the model
  private: physics::ModelPtr model;






};

}
#endif
