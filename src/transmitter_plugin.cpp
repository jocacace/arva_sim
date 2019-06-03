/*
 * Copyright (C) 2019, Jonathan Cacace.
 * Email id : jonathan.cacace@gmail.com
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include <ros/ros.h>
#include <gazebo/gazebo_client.hh>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <tf/transform_broadcaster.h>
#include <ctime>

using namespace std;
	
namespace gazebo
{
  class ArtvaTransmitter : public ModelPlugin
  {

	//Ros variables		
	private: ros::NodeHandle* _node_handle;
	private: clock_t begin; 
	ignition::math::Pose3d _obj_pose;
	private: physics::ModelPtr model;
	private: string _child_frame;
	private: string _frame_id;
	private: event::ConnectionPtr updateConnection;

	public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {	
		_node_handle = new ros::NodeHandle();	
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ArtvaTransmitter::OnUpdate, this));
		model = _parent;
		_child_frame = "arva_data_" + std::to_string(_sdf->Get<int>("id"));
		_frame_id = _sdf->Get<string>("frame_id");
		begin = clock();
	}

    // Called by the world update start event
    public: void OnUpdate()  {
			clock_t end = clock();
			if( (double(end - begin) / CLOCKS_PER_SEC) > 0.25 ) {
				static tf::TransformBroadcaster br;
				_obj_pose = this->model->WorldPose();
				tf::Transform transform;
				transform.setOrigin( tf::Vector3(_obj_pose.Pos()[0], _obj_pose.Pos()[1], _obj_pose.Pos()[2]) );
				tf::Quaternion q (_obj_pose.Rot().X(), _obj_pose.Rot().Y(), _obj_pose.Rot().Z(), _obj_pose.Rot().W()); 

				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _frame_id, _child_frame));
				begin = clock();
			}
		}
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ArtvaTransmitter)
}



