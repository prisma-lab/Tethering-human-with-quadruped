#include <string>
#include <vector>
#include <tf/tf.h>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <ignition/math.hh>
#include <Eigen/Core>
#include "gazebo_msgs/ModelStates.h"
#include <angles/angles.h>
#include "tf_conversions/tf_eigen.h"
#include "utils.hpp"
#include "geometry_msgs/Vector3.h"

#define WALKING_ANIMATION "walking"

using namespace std;
using namespace Eigen;

namespace gazebo {

  class GZ_PLUGIN_VISIBLE ActorPosPlugin : public ModelPlugin
    {     /// \brief Constructor
      public: ActorPosPlugin();

      // \brief Load the actor plugin.
      // \param[in] _model Pointer to the parent model.
      // \param[in] _sdf Pointer to the plugin's SDF elements.
      public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      public: void modelStateCallback(const gazebo_msgs::ModelStates & msg);

      // Documentation Inherited.
      public: virtual void Reset();

      // \brief Function that is called every update cycle.
      // \param[in] _info Timing information
      private: void OnUpdate(const common::UpdateInfo &_info);
      public: void human_vel_cb( const geometry_msgs::Twist & msg );
      public: void human_cmd_cb( const geometry_msgs::Pose & msg );

      // \brief Pointer to the parent actor.
      private: physics::ActorPtr actor;

      // \brief Pointer to the world, for convenience.
      private: physics::WorldPtr world;

      // \brief Pointer to the sdf element.
      private: sdf::ElementPtr sdf;

      private: ros::NodeHandle* _node_handle;
      private: ros::Publisher  _man_pos_pub;
      private: ros::Publisher  _human_hand_pos_pub;
      private: ros::Subscriber  _dog_pos_sub;
      private: ros::Subscriber _human_pose_sub;

      private: ros::Subscriber  _human_vel_sub;
      private: geometry_msgs::Twist _human_vel_data;


      // \brief Velocity of the actor
      private: ignition::math::Vector3d velocity;

      // \brief List of connections
      private: std::vector<event::ConnectionPtr> connections;

      // \brief Current target location
      private: ignition::math::Vector3d target;

      // \brief Target location weight (used for vector field)
      private: double targetWeight = 1.0;

      // \brief Obstacle weight (used for vector field)
      private: double obstacleWeight = 1.0;

      // \brief Time scaling factor. Used to coordinate translational motion
      /// with the actor's walking animation.
      private: double animationFactor = 1.0;

      // \brief Time of the last update.
      private: common::Time lastUpdate;

      // \brief List of models to ignore. Used for vector field
      private: std::vector<std::string> ignoreModels;

      // \brief Custom trajectory info.
      private: physics::TrajectoryInfoPtr trajectoryInfo;

      private: Eigen::Vector3d _dog_pos;
      private: ignition::math::Vector3d rpy_dog;
      private: Eigen::Matrix<double,3,3> _world_actor_base;
      private: bool _first_h_pose;

      private: geometry_msgs::Pose _human_pose_data;
   		private: physics:: ModelPtr actor_model;

    };

  /////////////////////////////////////////////////
  ActorPosPlugin::ActorPosPlugin()
  {
  }

  /////////////////////////////////////////////////
  void ActorPosPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

    this->sdf = _sdf;
    this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
    this->world = this->actor->GetWorld();
    actor_model = world->ModelByName("actor");
	  
    //ignition::math::Vector3d link_pose = (ignition::math::Vector3d(x,y,z)-visual_->Pose().Pos())*1000;   
    //ignition::math::Vector3d endpoint = (visual_->WorldPose().Rot().RotateVectorReverse(poles-visual_->WorldPose().Pos()))*1000 ;
	  //cout << "LeftHand: " << actor_model->GetLink( "LeftHand")->WorldPose().Pos().X() << " " <<
	  //						  actor_model->GetLink( "LeftHand")->WorldPose().Pos().Y() << " "  <<
	  //						  actor_model->GetLink( "LeftHand")->WorldPose().Pos().Z() << endl;
    
    _node_handle = new ros::NodeHandle();
    _man_pos_pub = _node_handle->advertise<geometry_msgs::PointStamped>("/actor_pos", 1);
    _dog_pos_sub = _node_handle->subscribe("/gazebo/model_states", 1, &ActorPosPlugin::modelStateCallback, this );
    _human_vel_sub = _node_handle->subscribe("/human/cmd_vel", 1, &ActorPosPlugin::human_vel_cb, this );
    _human_pose_sub = _node_handle->subscribe("/human/cmd_pos", 1, &ActorPosPlugin::human_cmd_cb, this);

    _human_hand_pos_pub = _node_handle->advertise<geometry_msgs::Vector3>("/human/hand_pose", 1);
    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
            std::bind(&ActorPosPlugin::OnUpdate, this, std::placeholders::_1)));


    this->Reset();


    _first_h_pose = false;

    // Read in the target weight
    if (_sdf->HasElement("target_weight"))
      this->targetWeight = _sdf->Get<double>("target_weight");
    else
      this->targetWeight = 1.15;

    // Read in the obstacle weight
    if (_sdf->HasElement("obstacle_weight"))
      this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
    else
      this->obstacleWeight = 1.5;

    // Read in the animation factor (applied in the OnUpdate function).
    if (_sdf->HasElement("animation_factor"))
      this->animationFactor = _sdf->Get<double>("animation_factor");
    else
      this->animationFactor = 4.5;

    // Add our own name to models we should ignore when avoiding obstacles.
    this->ignoreModels.push_back(this->actor->GetName());

    // Read in the other obstacles to ignore
    if (_sdf->HasElement("ignore_obstacles"))
    {
      sdf::ElementPtr modelElem =
        _sdf->GetElement("ignore_obstacles")->GetElement("model"); 
      while (modelElem)
      {
        this->ignoreModels.push_back(modelElem->Get<std::string>());
        modelElem = modelElem->GetNextElement("model");
      }
    }
  }

  ///////////////////////////////////////////////

  ///////////////////////////////////////////////
  void ActorPosPlugin::human_vel_cb( const geometry_msgs::Twist & msg ) {

    _human_vel_data = msg;

  }


  void ActorPosPlugin::human_cmd_cb( const geometry_msgs::Pose & msg ) {
    _human_pose_data = msg;
  }

  void ActorPosPlugin::modelStateCallback(const gazebo_msgs::ModelStates & msg) {


      bool found = false;
      int index = 0;
      std::string _human_model_name="actor";

      while( !found  && index < msg.name.size() ) {

          if( msg.name[index] == _human_model_name )
              found = true;
          else index++;
      }

      if( found ) {
          
        ignition::math::Quaterniond q2(msg.pose[index].orientation.w, msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z);
        rpy_dog=q2.Euler();
        
        _world_actor_base <<  cos( rpy_dog.Z() ), -sin( rpy_dog.Z() ), 0,
                sin( rpy_dog.Z() ), cos( rpy_dog.Z() ), 0,
                0, 0, 1;

        _dog_pos << msg.pose[index].position.x, msg.pose[index].position.y, msg.pose[index].position.z;
        _first_h_pose = true;
      }

  }

  ///////////////////////////////////////////////
  void ActorPosPlugin::Reset()
  {
    this->velocity = 0.05;
    this->lastUpdate = 0;

    if (this->sdf && this->sdf->HasElement("target"))
      this->target = this->sdf->Get<ignition::math::Vector3d>("target");
    else
      this->target = ignition::math::Vector3d(0, -5, 1.2138);

    auto skelAnims = this->actor->SkeletonAnimations();
    if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
    {
      gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
    }
    else
    {
      // Create custom trajectory
      this->trajectoryInfo.reset(new physics::TrajectoryInfo());
      this->trajectoryInfo->type = WALKING_ANIMATION;
      this->trajectoryInfo->duration = 1.0;

      this->actor->SetCustomTrajectory(this->trajectoryInfo);
    }
  }


  /////////////////////////////////////////////////
  void ActorPosPlugin::OnUpdate(const common::UpdateInfo &_info) {


    if( _first_h_pose ) {
      common::Time currTime = _info.simTime ;
      double dt = (_info.simTime - this->lastUpdate).Double();

      this->velocity = 0.07;
      
      ignition::math::Pose3d pose = this->actor->WorldPose();
  
      ignition::math::Vector3d pos;
      ignition::math::Vector3d rpy;
 

      pos = pos.Normalize() * this->targetWeight;
      Eigen::Vector3d b_vel; 
      b_vel << _human_vel_data.linear.x, 0.0, 0.0;
      b_vel = _world_actor_base.transpose()*b_vel;

      //pose.Pos().X() += -b_vel[1]*dt;
      //pose.Pos().Y() += -b_vel[0]*dt;      

      pose.Pos().X() = _human_pose_data.position.x;
      pose.Pos().Y() = _human_pose_data.position.y;

      double yaw_rad = rpy_dog.Z() + _human_vel_data.angular.z*dt;

      while( yaw_rad < 0.0 ) yaw_rad += 2*M_PI;
      while( yaw_rad > 2*M_PI ) yaw_rad -= 2*M_PI;
	
      Eigen::Vector4f q;
      q << _human_pose_data.orientation.w, _human_pose_data.orientation.x, 
      _human_pose_data.orientation.y, _human_pose_data.orientation.z; 
      
      Eigen::Matrix3f R = utilities::QuatToMat(q);
      Eigen::Vector3f laser_rpy;
      laser_rpy = utilities::R2XYZ(R);

      pose.Rot() = ignition::math::Quaterniond(1.5707, 0, laser_rpy[2] ); 
      ros::Time now= ros::Time::now();
  
 
      double distanceTraveled = (pose.Pos() - this->actor->WorldPose().Pos()).Length();

      this->actor->SetWorldPose(pose, false, false);
      //this->actor->SetWorldTwist(_human_vel_data, false, false);
      //this->actor->SetRelativePose(pose, false, false);

      this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * this->animationFactor));
      this->lastUpdate = _info.simTime;

      geometry_msgs::Vector3 pp;
      pp.x = actor_model->GetLink( "LeftHand")->WorldPose().Pos().X();
   	  pp.y = actor_model->GetLink( "LeftHand")->WorldPose().Pos().Y(); 
	 	  pp.z = actor_model->GetLink( "LeftHand")->WorldPose().Pos().Z();

      _human_hand_pos_pub.publish( pp );
    }  
  }

  GZ_REGISTER_MODEL_PLUGIN(ActorPosPlugin)
}