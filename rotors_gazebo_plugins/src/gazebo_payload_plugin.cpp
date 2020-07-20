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
    gzerr<< "TPosCallback\n";
    ignition::math::Pose3d tpose(pos->position().x(), pos->position().y(), pos->position().z(), 0, 0, 0);
    gzerr << pos->position().x();

    payload_->SetWorldPose(tpose);
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

    double distance_net = z_Axis.Dot(pos_err);

    //state machine

    if (distance_net < 0.1 && precatch_ == true){
        precatch_ = false;
        catching_ = true;
    }

    //ignition::math::Quaterniond rot_err = q_pr_pa_.Inverse()*pose_parent.Rot().Inverse()*pose_payload.Rot();

    double omega = 3*33.3;    // natural frequency
    double zeta = 1.0;      // damping ratio
    double mass = 1;
    double inertia = 0.4*mass*0.0025; // inertia of solid sphere: 0.4*m*r²
    double k_p_lin_z = omega*omega*mass;
    double k_d_lin_xy = 5 * zeta*omega*mass;
    double k_d_lin_z = 2*zeta*omega*mass;
    double reactio;

    if(precatch_ || pose_parent.Pos()[2] < 3) {
        k_p_lin_z = 0;
        k_d_lin_xy = 0;
        k_d_lin_z = 0;
        reactio = 0;

    } else if(catching_) {
        reactio = 1;

    } else {
        reactio = 1;
    }

    double forcex  = -k_d_lin_xy*lin_vel_err.Dot(x_Axis);
    double forcey  = -k_d_lin_xy*lin_vel_err.Dot(y_Axis);
    double forcez  = k_p_lin_z*pos_err.Dot(z_Axis)-k_d_lin_z*lin_vel_err.Dot(z_Axis);
    /*
    if(force.Length()>100)
        force = force/force.Length()*100;
    */
    gzerr << forcex << " " << forcey <<  " " << forcez << "\n";
    ignition::math::Vector3d force(forcex, forcey, forcez);

    force.Correct();

    /*
    gzdbg<<"x: "<<force.X()<<" y: "<<force.Y()<<" z: "<<force.Z()<<"\n";
    gzdbg<<"rx: "<<pos_err.X()<<" ry: "<<pos_err.Y()<<" rz: "<<pos_err.Z()<<"\n";
    gzdbg<<"vx: "<<lin_vel_err.X()<<" vy: "<<lin_vel_err.Y()<<" vz: "<<lin_vel_err.Z()<<"\n";
    */
    ignition::math::Vector3d force_position(pos_err.Dot(x_Axis), pos_err.Dot(y_Axis), 0);
    ignition::math::Vector3d zero(0,0,0);

    parent_->AddForceAtRelativePosition(-reactio*force, force_position);
    payload_->AddForceAtRelativePosition(force, zero);
}
