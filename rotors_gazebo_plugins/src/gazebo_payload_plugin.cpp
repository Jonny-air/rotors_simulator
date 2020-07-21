/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * Modifications by David Rohr, drohr@student.ethz.ch
 *
 */

#include "rotors_gazebo_plugins/gazebo_payload_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboPayloadPlugin)

/////////////////////////////////////////////////
GazeboPayloadPlugin::GazeboPayloadPlugin()
{
    gzdbg<<"GazeboPayloadPlugin constructed\n";
    payload_pos_old_[0] = -1;
    payload_pos_old_[1] = -1;
    payload_pos_old_[2] = -1;

    precatch_ = true;
    catching_ = false;
    postcatch_ = false;
}

/////////////////////////////////////////////////
GazeboPayloadPlugin::~GazeboPayloadPlugin()
{
    gzdbg<<"GazeboPayloadPlugin destructed\n";
}

/////////////////////////////////////////////////
void GazeboPayloadPlugin::Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf)
{
    gzdbg<<"load called"<<std::endl;

    GZ_ASSERT(_model, "GazeboPayloadPlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "GazeboPayloadPlugin _sdf pointer is NULL");

    this->model_ = _model;
    this->world_ = this->model_->GetWorld();
    GZ_ASSERT(this->world_, "GazeboPayloadPlugin world pointer is NULL");

    namespace_.clear();

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPayloadPlugin::OnUpdate, this));

    /*
    if (_sdf->HasElement("link")) {
        std::string link_name = _sdf->Get<std::string>("link");
        this->link_ = this->model_->GetLink(link_name);
    } else if (!this->link_) {
        gzerr << "Link not found\n";
    }
    */

    /*
    std::string joint_name;
    if (_sdf->HasElement("joint")) {
        joint_name = _sdf->Get<std::string>("joint");
        this->joint_ = this->model_->GetJoint(joint_name);
    }

    if (!this->joint_){
        gzerr<<"Joint "<<joint_name<<" not found \n";

    } else {
        payload_ = joint_->GetChild();
        parent_ = joint_->GetParent();
    }
    */

    if (_sdf->HasElement("payload")) {
        std::string payload = _sdf->Get<std::string>("payload");
        this->payload_ = this->model_->GetLink(payload);
    }

    if (_sdf->HasElement("parent")) {
        std::string parent = _sdf->Get<std::string>("parent");
        this->parent_ = this->model_->GetLink(parent);
    }

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr<<"Please specify a robotNamespace.\n";

    if (_sdf->HasElement("hoistPosParent"))
        hoist_pos_parent_ = _sdf->Get<ignition::math::Vector3d>("hoistPosParent");
    else
        hoist_pos_parent_ = ignition::math::Vector3d(0,0,0);

    if (_sdf->HasElement("hoistPosPayload"))
        hoist_pos_payload_ = _sdf->Get<ignition::math::Vector3d>("hoistPosPayload");
    else
        hoist_pos_payload_ = ignition::math::Vector3d(0,0,0);
}

/////////////////////////////////////////////////
void GazeboPayloadPlugin::CreatePubsAndSubs(){
    target_pos_sub_ = node_handle_->Subscribe("~/visualization/victim_position", &GazeboPayloadPlugin::TPosCallback, this);
    // target_pos_sub_->Subscribe(&GazeboPayloadPlugin::TPosCallback, this);

    // force_pub_ = node_handle_->Publish("/pigeon/interaction_force",  , this);

    gazebo::transport::PublisherPtr connect_ros_to_gazebo_topic_pub =
        node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
            "~/" + kConnectRosToGazeboSubtopic, 1);

    gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;
    connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/visualization/victim_position");
    connect_ros_to_gazebo_topic_msg.set_ros_topic("/visualization/victim_position");
    connect_ros_to_gazebo_topic_msg.set_msgtype(gz_std_msgs::ConnectRosToGazeboTopic::VICTIM_POSITION);
    connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg, true);
}

/////////////////////////////////////////////////
void GazeboPayloadPlugin::TPosCallback(Vector3dStampedPtr &pos){
  if(precatch_){
    ignition::math::Pose3d tpose(pos->position().x(), pos->position().y(), pos->position().z(), 0, 0, 0);
    double time = pos->header().stamp().sec() + double(pos->header().stamp().nsec()) / 1e9;
    if(payload_pos_old_[2] != -1){
      double dt = time - payload_time_old_;
      double vx = (pos->position().x() - payload_pos_old_[0])/dt;
      double vy = (pos->position().y() - payload_pos_old_[1])/dt;
      double vz = (pos->position().z() - payload_pos_old_[2])/dt;
      ignition::math::Vector3d v(vx, vy, vz);
      payload_->SetLinearVel(v);
    }
    payload_->SetWorldPose(tpose);

    payload_time_old_ = time;
    payload_pos_old_[0] = pos->position().x();
    payload_pos_old_[1] = pos->position().y();
    payload_pos_old_[2] = pos->position().z();
  }
}

/////////////////////////////////////////////////
void GazeboPayloadPlugin::OnUpdate() {
    if (!pubs_and_subs_created_) {
      CreatePubsAndSubs();
      pubs_and_subs_created_ = true;
    }
    common::Time current_time = world_->SimTime();

    ignition::math::Pose3d pose_parent = parent_->WorldPose();
    ignition::math::Pose3d pose_payload = payload_->WorldPose();
    //ignition::math::Pose3d pose_payload_reset;
    // pose_payload_reset.Pos() = pose_parent.Pos() + pose_parent.Rot().RotateVector(hoist_pos_parent_) - pose_payload.Rot().RotateVector(hoist_pos_payload_);
    // pose_payload_reset.Rot() = pose_parent.Rot();

    //pose_parent.Pos() += pose_parent.Rot().RotateVector(hoist_pos_parent_);
    //pose_payload.Pos() += pose_payload.Rot().RotateVector(hoist_pos_payload_);

    // if(ini_){
    //     q_pr_pa_ = pose_parent.Rot().Inverse()*pose_payload.Rot();
    //     ini_=false;
    // }

    // Position error quantities (expressed in world frame)
    ignition::math::Vector3d lin_vel_err =  payload_->WorldLinearVel(hoist_pos_payload_) - parent_->WorldLinearVel(hoist_pos_parent_);
    //ignition::math::Vector3d pos_err = pose_parent.Pos()-pose_payload.Pos();
    ignition::math::Vector3d pos_err = pose_parent.Pos()-pose_payload.Pos();
    lin_vel_err.Correct();

    // Orientation error quantities (expressed in payload frame)
    // ignition::math::Vector3d rot_vel_err = pose_payload.Rot().RotateVectorReverse(payload_->WorldAngularVel()-parent_->WorldAngularVel());


    ignition::math::Vector3d z_Axis = pose_parent.Rot().ZAxis();
    ignition::math::Vector3d y_Axis = pose_parent.Rot().YAxis();
    ignition::math::Vector3d x_Axis = pose_parent.Rot().XAxis();

    double distance_net = pos_err.Dot(z_Axis);

    //state machine

    if (distance_net > -0.1 && precatch_ && pose_payload.Pos()[2] > 2){
        precatch_ = false;
        catching_ = true;
    }

    //ignition::math::Quaterniond rot_err = q_pr_pa_.Inverse()*pose_parent.Rot().Inverse()*pose_payload.Rot();

    // double omega = 1;    // natural frequency
    // double zeta = 1.0;      // damping ratio
    // double mass = 1;
    double k_p_lin_z = 100;
    double k_d_lin_xy = 10;//5 * zeta*omega*mass;
    double k_d_lin_z = 10; //2;
    double reactio_xy = 0;
    double reactio_z = 1;

    if(precatch_) {
        k_p_lin_z = 0;
        k_d_lin_xy = 0;
        k_d_lin_z = 0;
        reactio_z = 0;

    } //else if(catching_) {
    //     reactio = 1;
    //
    // } else {
    //     reactio = 1;
    // }

    double forcex_t  = -k_d_lin_xy*lin_vel_err.Dot(x_Axis);
    double forcey_t  = -k_d_lin_xy*lin_vel_err.Dot(y_Axis);
    double forcez_t  = k_p_lin_z*pos_err[2]-k_d_lin_z*lin_vel_err[2]; //.Dot(z_Axis)-k_d_lin_z*lin_vel_err.Dot(z_Axis);

    double forcex_d = -reactio_xy * forcex_t;
    double forcey_d = -reactio_xy * forcey_t;
    double forcez_d = -reactio_z * forcez_t;
    // if(forcex>100)
    //     forcex = 100;
    // if(forcey>100)
    //     forcey = 100;
    // if(forcez>100)
    //     forcez = 100;
    // if(forcex<-100)
    //     forcex = -100;
    // if(forcey<-100)
    //     forcey = -100;
    // if(forcez<-100)
    //     forcez = -100;

    gzerr << /*" precatch: " << precatch_ << " Catching: " << catching_ << " postcatch: " << postcatch_ <<*/
        " Fx: " << forcex_t << " Fy: " << forcey_t <<  " Fz: " << forcez_t <<
        " LinVelErr: " << lin_vel_err <<
        " PosErr: " << pos_err  << /*" D_net: " << distance_net <<*/"\n";
    ignition::math::Vector3d force_t(forcex_t, forcey_t, forcez_t);
    ignition::math::Vector3d force_d(forcex_d, forcey_d, forcez_d);

    force_d.Correct();
    force_t.Correct();

    ignition::math::Vector3d force_position(pos_err.Dot(x_Axis), pos_err.Dot(y_Axis), 0);
    ignition::math::Vector3d zero(0,0,0);

    parent_->AddForceAtRelativePosition(force_d, force_position);
    payload_->AddForceAtRelativePosition(force_t, zero);
}
