#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <ignition/math.hh>
#include <Eigen/Core>
#include "gazebo_msgs/ModelStates.h"
#include <angles/angles.h>



#define WALKING_ANIMATION "walking"

namespace gazebo
{

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

    // \brief Helper function to choose a new target location
     private: void ChooseNewTarget();

    // \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    // \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
     private: void HandleObstacles(ignition::math::Vector3d &_pos);

    // \brief Pointer to the parent actor.
    private: physics::ActorPtr actor;

    // \brief Pointer to the world, for convenience.
    private: physics::WorldPtr world;

    // \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    private: ros::NodeHandle* _node_handle;
    private: ros::Publisher  _man_pos_pub;
    private: ros::Subscriber  _dog_pos_sub;
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
  };

/////////////////////////////////////////////////
ActorPosPlugin::ActorPosPlugin()
{
}

/////////////////////////////////////////////////
void ActorPosPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();
  
  _node_handle = new ros::NodeHandle();
  _man_pos_pub = _node_handle->advertise<geometry_msgs::PointStamped>("/actor_pos", 1);
  _dog_pos_sub = _node_handle->subscribe("/gazebo/model_states", 1, &ActorPosPlugin::modelStateCallback, this );


  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPosPlugin::OnUpdate, this, std::placeholders::_1)));


  this->Reset();

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

void ActorPosPlugin::modelStateCallback(const gazebo_msgs::ModelStates & msg) {


    bool found = false;
    int index = 0;
    std::string _model_name="dogbot";

    while( !found  && index < msg.name.size() ) {

        if( msg.name[index] == _model_name )
            found = true;
        else index++;
    }

    if( found ) {
        
        //_world_H_base.setIdentity();
        //
        ////quaternion
       // tf::Quaternion q(msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z,  msg.pose[index].orientation.w);
       // q.normalize();
       // Eigen::Matrix<double,3,3> rot;
      //  tf::matrixTFToEigen(tf::Matrix3x3(q),rot);
      ignition::math::Quaterniond q(msg.pose[index].orientation.w, msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z);
      rpy_dog=q.Euler();
        ////Roll, pitch, yaw
        //double roll, pitch, yaw;
       // tf::Matrix3x3(q).getRPY(roll_dog, pitch_dog, yaw_dog);

        /*if (yaw< 3.15 && yaw>0)
        {
          yaw =-6.28+yaw;

          //cout<<"yaw "<<yaw<<endl;
        }*/

        //Set base pos (position and orientation)
        _dog_pos << msg.pose[index].position.x, msg.pose[index].position.y, msg.pose[index].position.z;
         //cout<<"base_pos "<<rot<<endl;
        //Set transformation matrix
       // _world_H_base.block(0,0,3,3)= rot;
       // _world_H_base.block(0,3,3,1)= _base_pos.block(0,0,3,1);
//
       // //Set base vel
       // _base_vel << msg.twist[index].linear.x, msg.twist[index].linear.y, msg.twist[index].linear.z, msg.twist[index].angular.x, msg.twist[index].angular.y, msg.twist[index].angular.z;
       // _first_wpose = true;
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

///////////////////////////////////////////////
void ActorPosPlugin::ChooseNewTarget()
{
  ignition::math::Vector3d newTarget(this->target);
  while ((newTarget - this->target).Length() < 2.0)
  {
    newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
    newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
          - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
  }
  this->target = newTarget;
}
///////////////////////////////////////////////
void ActorPosPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void ActorPosPlugin::OnUpdate(const common::UpdateInfo &_info)
{

 common::Time currTime = _info.simTime ;

//cASE 1
// if(currTime<= 15)
// {
//  this->velocity = 0.07;
// }
// else if(currTime> 15 && currTime<40)
// {
//  this->velocity = 0.1;
// }
// else if(currTime> 40 && currTime<60)
// {
//  this->velocity = 0.05;
// }
// else if(currTime> 60)
// {
//  this->velocity = 0.0;
// }

 //cASE 2
 if(currTime<= 15)
 {
  this->velocity = 0.07;
 }
 else if(currTime> 15 && currTime<16)
 {
  this->velocity = 0.1;
 }
// else if(currTime> 40 && currTime<60)
// {
//  this->velocity = 0.05;
// }
 else if(currTime> 16 && currTime<36)
 {
  this->velocity = 0.1;
 }
  else if(currTime> 36 )
 {
  this->velocity = 0.1;
 }
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();
  
   ignition::math::Pose3d pose = this->actor->WorldPose();

   float differencey=pose.Pos().Y()-_dog_pos[1];
   float differencex=pose.Pos().X()-_dog_pos[0];
   float difference=sqrt(pow(differencey,2)+pow(differencex,2)); 
      
   double alfa=angles::normalize_angle_positive(atan2(differencex,differencey));
ignition::math::Vector3d pos;
ignition::math::Vector3d rpy;

/*if(currTime<10)
  {
  ignition::math::Vector3d pos_target;
  pos_target.X()=1;
  pos_target.Y()=-1;
  pos_target.Z()=1;
   pos = pos_target - pose.Pos();
   rpy = pose.Rot().Euler();
  }
  else if(currTime>10 && currTime<20)
  {
  ignition::math::Vector3d pos_target;
  pos_target.X()=-1;
  pos_target.Y()=-2;
  pos_target.Z()=1;
   pos = pos_target - pose.Pos();
  rpy = pose.Rot().Euler();
  }
    else if(currTime>20 && currTime<30)
  {
  ignition::math::Vector3d pos_target;
  pos_target.X()=1;
  pos_target.Y()=-3;
  pos_target.Z()=1;
   pos = pos_target - pose.Pos();
  rpy = pose.Rot().Euler();
  }*/

  if(currTime<26 || currTime>36)
  {
  ignition::math::Vector3d pos_target;
  pos_target.X()=(_dog_pos[0])+sin(alfa);
  pos_target.Y()=(_dog_pos[1])+cos(alfa);
  pos_target.Z()=1;
   pos = pos_target - pose.Pos();
   rpy = pose.Rot().Euler();
  }
  else if(currTime>26 && currTime<36)
  {
  ignition::math::Vector3d pos_target;
  pos_target.X()=(_dog_pos[0])-1;
  pos_target.Y()=(_dog_pos[1]);
  pos_target.Z()=1; 
  rpy = pose.Rot().Euler();
  pos = pos_target - pose.Pos();
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();
   pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+ yaw.Radian());
 
  }

 if( currTime>=36)
  {
     pose.Rot() = ignition::math::Quaterniond(1.5707, 0, 1.57); 
  }
  else if(currTime<26)
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()); 
  }

 // std::cout<<"angolo"<<yaw.Radian()<<std::endl;
//
  //double distance = pos.Length();

  // Choose a new target position if the actor has reached its current
  // target.
  //if (distance < 0.3)
  //{
  //  this->ChooseNewTarget();
  //  pos = this->target - pose.Pos();
  //}

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  //this->HandleObstacles(pos);
   // Compute the yaw orientation
  
  //CASE 1 E 2
  /*ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();*/

  // Rotate in place, instead of jumping.
  //if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  //{
  //  pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
  //      yaw.Radian()*0.001);
  //}
  //else
  //{
    pose.Pos() += pos * this->velocity * dt;
    //pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+ yaw.Radian());
  //}

  // Make sure the actor stays within bounds
  //pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
  //pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
  //pose.Pos().Z(1.2138);

  ros::Time now= ros::Time::now();

  geometry_msgs::PointStamped p ; 
  
     p.header.stamp.sec=now.sec;
     p.header.stamp.nsec=now.nsec;  
     
     p.point.x = pose.Pos().X();
     p.point.y = pose.Pos().Y();
     p.point.z = pose.Pos().Z();
  
  _man_pos_pub.publish( p );

  
  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();
//
  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}

    GZ_REGISTER_MODEL_PLUGIN(ActorPosPlugin)
}