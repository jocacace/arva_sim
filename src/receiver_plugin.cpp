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
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <string.h>
#include <std_msgs/Int32.h>
#include <boost/algorithm/string.hpp>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ctime>
#include <arva_sim/arva_data.h>
#include "eigen3/Eigen/Geometry"
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "arva_sim/arva.h"
#include "tf2_msgs/TFMessage.h"
#include <boost/algorithm/string.hpp> 

using namespace std;

#define SIMULATE_TRACE 1

using Eigen::MatrixXd;
using namespace Eigen;

namespace gazebo
{
	struct comp
	{
		template<typename T>
		bool operator()(const T& l, const T& r) const
		{
			if (l.second != r.second)
				return l.second < r.second;

			return l.first < r.first;
		}
	};

  class ArvaReceiver : public ModelPlugin
  {

			
		typedef struct ARVA_TRANSMITTER {
			int id;
			Vector3f p;
			Matrix3f R;
			bool updated;
			ARVA_TRANSMITTER() {
				id = -1;
				updated = true;
			}
		}ARVA_TRANSMITTER;

    private: event::ConnectionPtr updateConnection;



		private: vector<int> _arva_topic_id;

		//private: vector<transport::SubscriberPtr> arva_subs; //list of arva data
		private: vector< ros::Subscriber > _arva_subs;
		private: ros::NodeHandle* _node_handle;
		private: ros::Publisher _arva_pub;
		private: clock_t begin; 
    private: transport::NodePtr node;

		private: vector< ARVA_TRANSMITTER > _arva_sensors; //all the transmitters detected at each cycle
		private: vector< ARVA_TRANSMITTER > _arva_signal;	//the only transmitters to publish 0 < c < channels
		private: Vector3f _receiver_pos;
		private: Matrix3f _receiver_rot;
		private: math::Pose _obj_pose;
		private: int _channels;

		private: visualization_msgs::MarkerArray _arva_marker;
		private: ros::Publisher _arva_marker_pub;
		private: int _seq;
		private: string _frame_id;
		private: string _dir_path;
		private: ros::Subscriber _tf_sub;
		private: bool _new_s_data;
		private: int _wd_cnt;

		private: vector < vector<float> > _marker_colors;

		private: arva_sim::arva _detected_arva;

		public: ArvaReceiver()  {
			node = transport::NodePtr(new transport::Node());
			node->Init("default");  
		}

		public: visualization_msgs::Marker create_arrow_marker(int id, Vector3f p, geometry_msgs::Vector3 direction ) {
			
			Vector3f Hb;
			Hb[0] = direction.x;
			Hb[1] = direction.y;
			Hb[2] = direction.z;

			geometry_msgs::Point start_p;
			geometry_msgs::Point end_p;
			Hb /= Hb.norm();

			visualization_msgs::Marker arrow_marker;
			arrow_marker.header.frame_id = _frame_id;

			arrow_marker.header.stamp = ros::Time::now();

			arrow_marker.ns = "arva_receiver";
			arrow_marker.id = id;
			arrow_marker.type = visualization_msgs::Marker::ARROW;
			arrow_marker.action = visualization_msgs::Marker::ADD;
			arrow_marker.pose.orientation.x = 0.0;
			arrow_marker.pose.orientation.y = 0.0;
			arrow_marker.pose.orientation.z = 0.0;
			arrow_marker.pose.orientation.w = 1.0;
			arrow_marker.scale.x = 0.1;
			arrow_marker.scale.y = 0.1;
			arrow_marker.scale.z = 0.1;

			arrow_marker.color.a = 1.0;
			arrow_marker.color.r = _marker_colors[id][0];
			arrow_marker.color.g = _marker_colors[id][1];
			arrow_marker.color.b = _marker_colors[id][2];
			arrow_marker.points.resize(2);

			start_p.x = 0.0; 
			start_p.y = 0.0; 
			start_p.z = 0.0; 
	
			end_p.x = Hb[0]; 
			end_p.y = Hb[1]; 
			end_p.z = Hb[2]; 
			arrow_marker.points[0] = start_p;
			arrow_marker.points[1] = end_p;


			return arrow_marker;
			
		}


		public: void get_arva_data(Vector3f p, Matrix3f R,Vector3f pt, Matrix3f Rt, Vector3f & Hb, float & d, float & delta ) {
			
			Vector3f m_vec(1.0, 0.0, 0.0);

			m_vec = Rt*m_vec;
			float r_noise = 50; //% [m] 50 
			float n_H_noise = 1 / pow(r_noise, 3) * (1 / (4*M_PI))*1.5420;
		
			Vector3f r;
			r = p - pt;

			Matrix3f A;
			A << 	2*pow(r[0],2)-pow(r[1],2)-pow(r[2],2), 3*r[0]*r[1], 3*r[0]*r[2],
					3*r[0]*r[1], 2*pow(r[1],2)-pow(r[0],2)-pow(r[2],2), 3*r[1]*r[2],
					3*r[0]*r[2], 3*r[1]*r[2], 2*pow(r[2],2)-pow(r[0],2)-pow(r[1],2);

			Vector3f Am( A(0,0)*m_vec[0]+A(0,1)*m_vec[1]+A(0,2)*m_vec[2],
								 A(1,0)*m_vec[0]+A(1,1)*m_vec[1]+A(1,2)*m_vec[2],
								 A(2,0)*m_vec[0]+A(2,1)*m_vec[1]+A(2,2)*m_vec[2]);
	
		
			float Am_x = A(0,0)*m_vec[0]+A(0,1)*m_vec[1]+A(0,2)*m_vec[2];
			float Am_y = A(1,0)*m_vec[0]+A(1,1)*m_vec[1]+A(1,2)*m_vec[2];
			float Am_z = A(2,0)*m_vec[0]+A(2,1)*m_vec[1]+A(2,2)*m_vec[2];

			float rd = r.norm();
	
			Vector3f H(1/(4*M_PI*pow(rd,5))*Am_x, 1/(4*M_PI*pow(rd,5))*Am_y, 1/(4*M_PI*pow(rd,5))*Am_z);
			Hb = R*H; 
		
			float d1 = powf( (1.0/fabs(Hb[0])*(1.0/(4.0*M_PI))*1.5420), 0.33);
			float d3 = powf( (1.0/Hb.norm()*(1.0/(4.0*M_PI))*1.5420), 0.33);


			if ( d3 <= 3.0 ) {
				d = d3;
				delta = 0.0;
			}
			else if( d1 <= r_noise ) {
				d = d1;
				delta = atan2(Hb[1], Hb[0]);
			}
			else {
				d = -1;
				delta = 0;
			}

			if( d!=-1) {
				d = round(d*100);
				delta = round( delta*180.0/M_PI);
			}
			
	}


	public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf ) {

		if (!ros::isInitialized()) {
			ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. " << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
			return;
		}

		this->model = _parent;

		_channels = _sdf->Get<int>("channels");
		_frame_id = _sdf->Get<string>("frame_id");

		_node_handle = new ros::NodeHandle();
		_tf_sub = _node_handle->subscribe("/tf", 0, &ArvaReceiver::tf_cb, this );
		_arva_pub = _node_handle->advertise<arva_sim::arva>("/arva", 0);
		_arva_marker_pub = _node_handle->advertise<visualization_msgs::MarkerArray>("/arva_transmitter", 0);

		begin = clock();

		_arva_signal.resize(_channels);
		_detected_arva.arva_signals.resize( _channels );


		vector< float > rand_color; rand_color.resize(3);
  	srand (time(NULL));
		for(int i=0; i<_channels; i++ ) {
			rand_color[0] = ((double) rand() / (RAND_MAX)) + 1;
			rand_color[1] = ((double) rand() / (RAND_MAX)) + 1;
			rand_color[2] = ((double) rand() / (RAND_MAX)) + 1;


			_marker_colors.push_back( rand_color );
		}
		_seq = 0;

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ArvaReceiver::loop, this));

		_wd_cnt = 0;
		_new_s_data = false;
	}


	public: void tf_cb( tf2_msgs::TFMessage tf_data ) {

		bool arva_found = false;
		for( int i=0; i<_arva_sensors.size(); i++ ) 
			_arva_sensors[i].updated = false;

		
		for( int i=0; i<_detected_arva.arva_signals.size(); i++ ) {
			_detected_arva.arva_signals[i].id = -1;
			_detected_arva.arva_signals[i].direction.x = -1;
			_detected_arva.arva_signals[i].direction.y = -1;
			_detected_arva.arva_signals[i].direction.z = -1;
		} //
		
		_arva_marker.markers.clear();

		for(int i=0; i<tf_data.transforms.size(); i++ ) {
		
		
			vector<string> result; 
			std::size_t found = tf_data.transforms[i].child_frame_id.find("arva_data_");
			if (found!=std::string::npos) {
				boost::split(result, tf_data.transforms[i].child_frame_id, boost::is_any_of("_"));
				
				if( result.size() == 3 ) {		

					stringstream ss(result[result.size()-1]);
					int id = 0; 
					ss >> id; 

					bool find = false;
					int index = 0;
					int el_id = -1;
					while (!find && index < _arva_sensors.size() ) {
						if( _arva_sensors[index].id == id ) {
							el_id = index;
							find = true;	
						}
						index++;
					}
					if (!find) {
						ARVA_TRANSMITTER transmitter;
						transmitter.id = id;
						transmitter.p[0] = tf_data.transforms[i].transform.translation.x;
						transmitter.p[1] = tf_data.transforms[i].transform.translation.y;
						transmitter.p[2] = tf_data.transforms[i].transform.translation.z;
						Quaternionf q;
						q.x() = tf_data.transforms[i].transform.rotation.x;
						q.y() = tf_data.transforms[i].transform.rotation.y;
						q.z() = tf_data.transforms[i].transform.rotation.z;
						q.w() = tf_data.transforms[i].transform.rotation.w;
						transmitter.R = q.normalized().toRotationMatrix();
						transmitter.updated = true;
						_arva_sensors.push_back( transmitter );

					}
					else {
						_arva_sensors[el_id].p[0] = tf_data.transforms[i].transform.translation.x;
						_arva_sensors[el_id].p[1] = tf_data.transforms[i].transform.translation.y;
						_arva_sensors[el_id].p[2] = tf_data.transforms[i].transform.translation.z;
						Quaternionf q;
						q.x() = tf_data.transforms[i].transform.rotation.x;
						q.y() = tf_data.transforms[i].transform.rotation.y;
						q.z() = tf_data.transforms[i].transform.rotation.z;
						q.w() = tf_data.transforms[i].transform.rotation.w;
						_arva_sensors[el_id].updated = true;
						_arva_sensors[el_id].R = q.normalized().toRotationMatrix();						
					}
					arva_found = true;		
				}
			}
		}
	
		if( arva_found ) {

			//Elab data
			_obj_pose = this->model->GetWorldPose();

			//---Receiver pose
			_receiver_pos[0] = _obj_pose.pos.x;
			_receiver_pos[1] = _obj_pose.pos.y;
			_receiver_pos[2] = _obj_pose.pos.z;
			float roll = _obj_pose.rot.GetRoll();
			float pitch = _obj_pose.rot.GetPitch();
			float yaw = _obj_pose.rot.GetYaw();
			Quaternionf q;
			q = AngleAxisf(roll, Vector3f::UnitX()) * AngleAxisf(pitch, Vector3f::UnitY()) * AngleAxisf(yaw, Vector3f::UnitZ());
			Eigen::Matrix3f Rt = q.normalized().toRotationMatrix();
			//---

			Vector3f Hb;
			float d;
			float  delta;
			std::map<int, float> sensors_map;

			//Calculate all the magnetic field flows
			vector <arva_sim::arva_data> swap_arva;
			for(int i=0; i<_arva_sensors.size(); i++ ) {
				get_arva_data( _arva_sensors[i].p, _arva_sensors[i].R, _receiver_pos, Rt, Hb, d, delta);
				arva_sim::arva_data sig;
				sig.id = _arva_sensors[i].id;
				sig.distance = d;
				sig.direction.x = Hb[0]; sig.direction.y = Hb[1]; sig.direction.z = Hb[2];
				swap_arva.push_back(sig);
				sensors_map.insert ( std::pair<int,float>(_arva_sensors[i].id, d ) );
			}

			std::set<std::pair<int, float>, comp> set(sensors_map.begin(), sensors_map.end());
			int n_elem = 0;
			for (auto const &pair: set) {
				if ( n_elem < _channels ) {
					for(int i=0; i<swap_arva.size(); i++) {
						if( swap_arva[i].id == pair.first ) {
							_detected_arva.arva_signals[n_elem] = swap_arva[i];
						}
					}
					n_elem++;	 
				}		
				else 
					continue;
			}

			_detected_arva.header.seq = _seq++;
			_detected_arva.header.stamp = ros::Time::now();
			_detected_arva.header.frame_id = _frame_id;



			for( int i=0; i<_detected_arva.arva_signals.size(); i++ ) {	
				if( _detected_arva.arva_signals[i].id != -1 ) {
					_arva_marker.markers.push_back( create_arrow_marker( i, _receiver_pos, _detected_arva.arva_signals[i].direction ) );
				}
			}


			_arva_marker_pub.publish( _arva_marker );

			_arva_pub.publish( _detected_arva );
			_new_s_data = true;

			
		}
	}


	// Called by the world update start event
	public: void loop() {
	
		clock_t end = clock(); //elapsed time
		if( (double(end - begin) / CLOCKS_PER_SEC) > 0.25 ) {

			//for( int i=0; i<_arva_sensors.size(); i++ ) {	
			//	if( !_arva_sensors[i].updated )
			//		_arva_sensors.erase( _arva_sensors.begin()+i);
			//}


			if(!_new_s_data ) {
				if( _wd_cnt++ > 2) { 
					_detected_arva.header.seq = _seq++;
					_detected_arva.header.stamp = ros::Time::now();
					_detected_arva.header.frame_id = _frame_id;
					for( int i=0; i<_detected_arva.arva_signals.size(); i++ ) {
						_detected_arva.arva_signals[i].id = -1;
						_detected_arva.arva_signals[i].direction.x = -1;
						_detected_arva.arva_signals[i].direction.y = -1;
						_detected_arva.arva_signals[i].direction.z = -1;
					} 
					_arva_pub.publish( _detected_arva );
				}
			}
			else _wd_cnt = 0;

			_new_s_data = false;
			begin = clock();
		}

	}
	
	// Pointer to the model
  private: physics::ModelPtr model;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ArvaReceiver)
}
