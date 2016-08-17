/**
 * @brief Estimator bridge plugin
 * @file estimator_bridge.cpp
 * @author Mohammed Kabir <kabir@uasys.io>
 *
 * Copyright 2016 Mohammed Kabir, UASys Pty Ltd.
 *
 * This will publish :
 * FCU global att quaternion (for compass hdg)
 * FCU raw GPS + covariance
 *
 * This will recieve (and send to FCU) :
 * VIO estimate
 *
 * It will also set a global-local reference origin and send transformed raw GPS meas to Rovio
 * and send reference origin to FCU for appropriate transformations on FCU side.
 */

#include <cmath>
#include <mavros/mavros_plugin.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

extern "C" {
  #include "libswiftnav/coord_system.h"
}

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#define MIN_NUM_SATS 14
#define EPH_THRESHOLD 3.0f
#define EPV_THRESHOLD 5.0f

namespace mavros {
namespace extra_plugins {

static constexpr double RAD_TO_DEG = 180.0 / M_PI;
static constexpr double DEG_TO_RAD = M_PI / 180.0;

// Frame transformation
static const Eigen::Quaterniond BODY_ROTATE_Q = ftf::quaternion_from_rpy(M_PI, 0.0, 0.0);
static const Eigen::Quaterniond GLOBAL_ROTATE_Q = ftf::quaternion_from_rpy(M_PI, 0.0, M_PI_2);

static const Eigen::Affine3d BODY_ROTATE_AFFINE(BODY_ROTATE_Q);
static const Eigen::Affine3d GLOBAL_ROTATE_AFFINE(GLOBAL_ROTATE_Q);

/**
 * @brief Estimator bridge plugin
 */
class EstimatorBridgePlugin : public plugin::PluginBase {
public:
	EstimatorBridgePlugin() : PluginBase(),
		estimator_bridge_nh("~estimator_bridge"),
		imu_inited(false),
		gps_inited(false),
		vio_inited(false),
		smoothing_factor(1.0),
		vio_init_count(0),
		should_reset_sp(false)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		double linear_stdev, angular_stdev, orientation_stdev, mag_stdev;

		estimator_bridge_nh.param("linear_acceleration_stdev", linear_stdev, 0.0003);
		estimator_bridge_nh.param("angular_velocity_stdev", angular_stdev, 0.02 * (M_PI / 180.0));
		estimator_bridge_nh.param("imu_lowpass_factor", smoothing_factor, 0.5);

		setup_covariance(linear_acceleration_cov, linear_stdev);
		setup_covariance(angular_velocity_cov, angular_stdev);
		setup_covariance(unk_orientation_cov, 0.0);

		fcu_gps_pub = estimator_bridge_nh.advertise<nav_msgs::Odometry>("raw_gps", 10);
		fcu_imu_pub = estimator_bridge_nh.advertise<sensor_msgs::Imu>("raw_imu", 10);
		fcu_state_pub = estimator_bridge_nh.advertise<geometry_msgs::PoseStamped>("onboard_state", 10);

		vio_odometry_sub = estimator_bridge_nh.subscribe("vio_estimate", 10, &EstimatorBridgePlugin::vio_odometry_cb, this);
		sp_reset_sub = estimator_bridge_nh.subscribe("reset_setpoint", 10, &EstimatorBridgePlugin::reset_sp_cb, this);

		Eigen::Vector3d k(1.0, 2.0, 3.0);
		ROS_INFO("INPUT, %f %f %f" , k.x(), k.y(), k.z());
		k = transform_vector_global(k);
		ROS_INFO("OUTPUT, %f %f %f" , k.x(), k.y(), k.z());

	}

	Subscriptions get_subscriptions() {
		return {
			       make_handler(&EstimatorBridgePlugin::handle_attitude_quaternion),
			       make_handler(&EstimatorBridgePlugin::handle_local_position_ned),
			       make_handler(&EstimatorBridgePlugin::handle_highres_imu),
			       make_handler(&EstimatorBridgePlugin::handle_gps_input)
		};
	}

private:
	ros::NodeHandle estimator_bridge_nh;
	std::string frame_id;

	ros::Publisher fcu_state_pub;
	ros::Publisher fcu_imu_pub;
	ros::Publisher fcu_gps_pub;

	ros::Subscriber vio_odometry_sub;
	ros::Subscriber sp_reset_sub;

	geometry_msgs::Point ENU_reference;
	sensor_msgs::NavSatFix LLH_datum;

	bool imu_inited;
	bool gps_inited;
	bool vio_inited;
	
	double smoothing_factor;
	Eigen::Vector3d filtered_gyro;
	Eigen::Vector3d filtered_accel;

	Eigen::Vector3d last_position;
	
	bool should_reset_sp;
	
	int vio_init_count;
	
	ros::Time last_vio_timestamp;

	ftf::Covariance3d linear_acceleration_cov;
	ftf::Covariance3d angular_velocity_cov;
	ftf::Covariance3d unk_orientation_cov;

	/* -*- low level helpers -*- */

	void setup_covariance(ftf::Covariance3d &cov, double stdev)
	{
		std::fill(cov.begin(), cov.end(), 0.0);
		if (stdev == 0.0)
			cov[0] = -1.0;
		else {
			cov[0 + 0] = cov[3 + 1] = cov[6 + 2] = std::pow(stdev, 2);
		}
	}

	Eigen::Quaterniond transform_quaternion_body(const Eigen::Quaterniond &q)
	{
		return q * BODY_ROTATE_Q;
	}

	Eigen::Vector3d transform_vector_body(const Eigen::Vector3d &vec)
	{
		return BODY_ROTATE_AFFINE * vec;
	}

	Eigen::Quaterniond transform_quaternion_global(const Eigen::Quaterniond &q)
	{
		return GLOBAL_ROTATE_Q * q;
	}

	Eigen::Vector3d transform_vector_global(const Eigen::Vector3d &vec)
	{
		return GLOBAL_ROTATE_AFFINE * vec;
	}

	/*
	 *	Incoming Data Handlers.
	 *	Handles raw data coming from the flight controller.
	 *
	 */

	// Flight controller global attitude estimate
	void handle_attitude_quaternion(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ATTITUDE_QUATERNION &att_q)
	{	
		
		auto pose_msg = boost::make_shared<geometry_msgs::PoseStamped>();

		// fill
		pose_msg->header = m_uas->synchronized_header("base_link", att_q.time_boot_ms);

		auto attitude = transform_quaternion_body(transform_quaternion_global(Eigen::Quaterniond(att_q.q1, att_q.q2, att_q.q3, att_q.q4)));
		tf::quaternionEigenToMsg(attitude, pose_msg->pose.orientation);

		tf::pointEigenToMsg(last_position, pose_msg->pose.position);

		// publish
		fcu_state_pub.publish(pose_msg);
	
	}

	void handle_local_position_ned(const mavlink::mavlink_message_t *msg, mavlink::common::msg::LOCAL_POSITION_NED &lpos)
	{	
		// Just store ENU position information
		last_position = transform_vector_global(Eigen::Vector3d(lpos.x, lpos.y, lpos.z));
	}

	// Flight controller raw inertial data
	void handle_highres_imu(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HIGHRES_IMU &imu_hr)
	{
		// check for VIO timeout
		double vio_delta = ros::Time::now().toSec() - last_vio_timestamp.toSec();
		if(vio_inited && vio_delta > 1.0) { 
			ROS_ERROR("EB : VIO deinitialized");
			if(gps_inited)ROS_ERROR("EB : GPS deinitialized");
			vio_inited = false;
			gps_inited = false;
		}
		
		if (imu_hr.fields_updated & ((7 << 3) | (7 << 0))) {

			auto raw_gyro = transform_vector_body(Eigen::Vector3d(imu_hr.xgyro, imu_hr.ygyro, imu_hr.zgyro));
			auto raw_accel = transform_vector_body(Eigen::Vector3d(imu_hr.xacc, imu_hr.yacc, imu_hr.zacc));
			
			auto imu_msg = boost::make_shared<sensor_msgs::Imu>();

			imu_msg->header = m_uas->synchronized_header(frame_id, imu_hr.time_usec);
			
			/* Lowpass the raw measurements
			if (!imu_inited) {
        		filtered_gyro = raw_gyro;
        		filtered_accel = raw_accel;
       			imu_inited = true;
    		} else {
       			for (int i = 0; i < 3; i++) {
           			filtered_gyro[i] = smoothing_factor * raw_gyro[i] + (1.0 - smoothing_factor) * raw_gyro[i];
           			filtered_accel[i] = smoothing_factor * raw_accel[i] + (1.0 - smoothing_factor) * raw_accel[i];
       			}	
   			}
    		*/

			tf::vectorEigenToMsg(raw_gyro, imu_msg->angular_velocity);
			tf::vectorEigenToMsg(raw_accel, imu_msg->linear_acceleration);

			imu_msg->angular_velocity_covariance = angular_velocity_cov;
			imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

			fcu_imu_pub.publish(imu_msg);
		}
	}

	// Flight controller raw GPS data
	void handle_gps_input(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_INPUT &gps)
	{
	
		auto fix = boost::make_shared<sensor_msgs::NavSatFix>();

		fix->latitude = gps.lat * 10000000.0f;		// to deg
		fix->longitude = gps.lon * 10000000.0f;		// to deg
		fix->altitude = gps.alt;					// m
		
		double ecef_datum[3];
		
		bool gps_good = ((gps.fix_type > 2)
			      && (gps.satellites_visible > MIN_NUM_SATS)
			      && (gps.vert_accuracy < EPV_THRESHOLD)
		    	  && (gps.horiz_accuracy < EPH_THRESHOLD));
		
		// GPS conversion to local frame
		if (!gps_inited && gps_good && vio_inited)
		{
			gps_inited = true;

			// Set datum such that GPS origin (datum) is ENU frame origin
			Eigen::Vector3d enu_origin;
			tf::pointMsgToEigen(ENU_reference, enu_origin);
			tf::pointEigenToMsg(-enu_origin, ENU_reference);

			{
				// Convert reference LLH-formatted datum to ECEF format
				double llh[3] = {fix->latitude * DEG_TO_RAD,
					  	 fix->longitude * DEG_TO_RAD,
					  	 fix->altitude };
				wgsllh2ecef(llh, ecef_datum);

				// Prepare NED vector from ENU coordinates, perform conversion in libswiftnav
				double ned[3] = { ENU_reference.y, ENU_reference.x, -ENU_reference.z };

				double ecef[3];
				wgsned2ecef_d(ned, ecef_datum, ecef);

				double llh_raw[3];
				wgsecef2llh(ecef, llh_raw);

				// Output datum. Convert radian latlon output back to degrees.
				LLH_datum.latitude = llh_raw[0] * RAD_TO_DEG;
				LLH_datum.longitude = llh_raw[1] * RAD_TO_DEG;
				LLH_datum.altitude = llh_raw[2];
			}
			
			// broadcast GPS origin for FCU
			mavlink::common::msg::GPS_GLOBAL_ORIGIN origin {};

			origin.latitude = LLH_datum.latitude * 10000000.0f;
			origin.longitude = LLH_datum.longitude * 10000000.0f;
			origin.altitude = LLH_datum.altitude * 1000.0f;

			UAS_FCU(m_uas)->send_message(origin);
			
			ROS_INFO("EB: GPS initialized");
			ROS_INFO("EB: Global reference - Latitude: %f, Longitude: %f, Altitude: %f m", LLH_datum.latitude, LLH_datum.longitude, LLH_datum.altitude);
			ROS_INFO("EB: Local reference - x: %f m, y: %f m, z: %f m", ENU_reference.x, ENU_reference.y, ENU_reference.z);

		}
		
		if (!gps_inited)
		{
			if(vio_inited) ROS_WARN_THROTTLE_NAMED(30, "estimator_bridge", "EB: Waiting for GPS fix");
			return;
		}
		if (gps_inited && !gps_good) {
			ROS_WARN_THROTTLE_NAMED(15, "estimator_bridge", "EB: Bad GPS quality, %d sats visible", gps.satellites_visible);
			gps_inited = false;
			ROS_ERROR("EB : GPS deinitialized");
			return;
		}

		auto gps_odom_msg = boost::make_shared<nav_msgs::Odometry>();

		gps_odom_msg->header = m_uas->synchronized_header("map", gps.time_usec);;

		// Set unknown fields
		gps_odom_msg->pose.pose.orientation.x = 0;
		gps_odom_msg->pose.pose.orientation.y = 0;
		gps_odom_msg->pose.pose.orientation.z = 0;
		gps_odom_msg->pose.pose.orientation.w = 1;
		gps_odom_msg->twist.twist.angular.x = 0;
		gps_odom_msg->twist.twist.angular.y = 0;
		gps_odom_msg->twist.twist.angular.z = 0;

		// Transform from global coordinates to local coordinates
		{
			// Convert reference LLH-formatted datum to ECEF format
			double llh_datum[3] = {LLH_datum.latitude * DEG_TO_RAD,
				  	       LLH_datum.longitude * DEG_TO_RAD,
				  	       LLH_datum.altitude };
			wgsllh2ecef(llh_datum, ecef_datum);		
		
			// Prepare the appropriate input vector to convert the input latlon
			// to an ECEF triplet.
			double llh[3] = { fix->latitude * DEG_TO_RAD,
					  		  fix->longitude * DEG_TO_RAD,
					  		  fix->altitude };
			double ecef[3];
			wgsllh2ecef(llh, ecef);

			// ECEF triplet is converted to north-east-down (NED), by combining it
			// with the ECEF-formatted datum point.
			double ned[3];
			wgsecef2ned_d(ecef, ecef_datum, ned);

			// Output data
			gps_odom_msg->pose.pose.position.x = ned[1];
			gps_odom_msg->pose.pose.position.y = ned[0];
			gps_odom_msg->pose.pose.position.z = -ned[2];
			
		}
		
		auto vel = transform_vector_global(Eigen::Vector3d(gps.vn,
														   gps.ve,
														   gps.vd));

		tf::vectorEigenToMsg(vel, gps_odom_msg->twist.twist.linear);

		// Covariances
		//for(int i = 0; i < 36; i++)
		//{
			// Set unknown covariances
		//	gps_odom_msg->pose.covariance[i] = 1e6f;
		//	gps_odom_msg->twist.covariance[i] = 1e6f;
		//}
		
		// variances are std-deviation squared
		gps_odom_msg->pose.covariance[0] = gps.horiz_accuracy * gps.horiz_accuracy;
		gps_odom_msg->pose.covariance[7] = gps.horiz_accuracy * gps.horiz_accuracy;
		gps_odom_msg->pose.covariance[14] = gps.vert_accuracy * gps.vert_accuracy;;
		gps_odom_msg->twist.covariance[0] = gps.speed_accuracy * gps.speed_accuracy;
		gps_odom_msg->twist.covariance[7] = gps.speed_accuracy * gps.speed_accuracy;
		gps_odom_msg->twist.covariance[14] = gps.speed_accuracy * gps.speed_accuracy;

		gps_odom_msg->child_frame_id = "base_link";

		fcu_gps_pub.publish(gps_odom_msg);
		
	}

	/*
	 *	Outgoing Data Handlers.
	 *	Sends fused state estimate from VIO to the flight controller.
	 *
	 *	TODO :
	 *	1. Fill in accelerations from VIO
	 * 	2. Fill in attitude covariances
	 *	4. Properly fill covariance of pos/vel/acc
	 *	5. Implement controller setpoint resets
	 *  6. Implement VIO reset from FC
	 *
	 */

	void vio_odometry_cb(const nav_msgs::Odometry::ConstPtr &odom) {
		
		if (!gps_inited) {
			// Save ENU frame reference till GPS gets initial fix.
			ENU_reference = odom->pose.pose.position;
		}
		
		if(!vio_inited && vio_init_count > 30)
		{
			vio_inited = true;
			vio_init_count = 0;
			ROS_INFO("EB: VIO initialized ");
		} else if (!vio_inited){
			vio_init_count++;
			return;
		}
		last_vio_timestamp = odom->header.stamp;
		
		mavlink::common::msg::ATTITUDE_QUATERNION_COV att {};
		mavlink::common::msg::LOCAL_POSITION_NED_COV local {};

		att.time_usec = odom->header.stamp.toNSec() / 1000;
		local.time_usec = odom->header.stamp.toNSec() / 1000;

		Eigen::Affine3d pose;
		tf::poseMsgToEigen(odom->pose.pose, pose);
		Eigen::Vector3d vel;
		tf::vectorMsgToEigen(odom->twist.twist.linear, vel);
		Eigen::Vector3d ang;
		tf::vectorMsgToEigen(odom->twist.twist.angular, ang);

		auto quaternion = transform_quaternion_global(transform_quaternion_body(Eigen::Quaterniond(pose.rotation())));
		auto angular_velocity = transform_vector_body(ang);
		auto position = transform_vector_global(Eigen::Vector3d(pose.translation()));
		auto linear_velocity = transform_vector_global(vel);
		
		// Fill in ATTITUDE_QUATERNION_COV
		mavros::ftf::quaternion_to_mavlink(quaternion, att.q);

		att.rollspeed = angular_velocity.x();
		att.pitchspeed = angular_velocity.y();
		att.yawspeed = angular_velocity.z();
		
		// Fill in LOCAL_POSITION_NED_COV
		local.estimator_type = (uint8_t)mavlink::common::MAV_ESTIMATOR_TYPE::VIO;
		
		local.x = position.x();
		local.y = position.y();
		local.z = position.z();

		local.vx = linear_velocity.x();
		local.vy = linear_velocity.y();
		local.vz = linear_velocity.z();
		
		// Covariance matrix (only diagnoal for now)
		// Position
		local.covariance[0] = odom->pose.covariance[7];
		local.covariance[9] = odom->pose.covariance[0];
		local.covariance[17] = odom->pose.covariance[14];
		// Velocity
		local.covariance[24] = odom->twist.covariance[7];
		local.covariance[30] = odom->twist.covariance[0];
		local.covariance[35] = odom->twist.covariance[14];
		// Acceleration
		// local.covariance[39] =
		// local.covariance[42] =
		// local.covariance[44] =

		// Send messages
		UAS_FCU(m_uas)->send_message_ignore_drop(att);
		UAS_FCU(m_uas)->send_message_ignore_drop(local);
	}
	
	void reset_sp_cb(const std_msgs::Empty::ConstPtr &msg) {
		should_reset_sp = true;
		if(vio_inited) ROS_ERROR("EB : VIO deinitialized");
		vio_inited = false;
		vio_init_count = 0;
		should_reset_sp = false;
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::EstimatorBridgePlugin, mavros::plugin::PluginBase)

