
#include <particle_filter/particle_filter.h>
#include <particle_filter/Particle_vector.h>
#include <particle_filter/Pose.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <eigen3/Eigen/Geometry>

//Particle filter. 
ParticleFilter *pf = 0;  

//Odometry. 
nav_msgs::Odometry *odom = 0;

//IMU.
sensor_msgs::Imu *imu = 0; 
double imu_yaw; 

//Keeps GPS reading. 
particle_filter::Pose *gps_pose = 0; 

bool can_start() {
	return (odom && gps_pose && pf->calibrated()); 
}

double get_yaw_from_quaternion(Eigen::Vector4d reading) {
	//Gets quaternion values.
	double x = reading(0); 
	double y = reading(1);
	double z = reading(2);
	double w = reading(3); 

	Eigen::Quaternion<double> quaternion(x, y, z, w); 

	//Gets rotation matrix from quaternion.
	Eigen::Matrix<double, 3, 3> rotation_matrix = quaternion.normalized().toRotationMatrix(); 

	//Gets Euler angles from the matrix. 
	Eigen::Matrix<double, 3, 1> angles = rotation_matrix.eulerAngles(2, 1, 0); 

	//Returns yaw (i.e. rotation in heading).  
	return angles(2); 
}

void gps_callback(particle_filter::Pose gps_reading) {
	//Initializes pose if first reading. 
	if (!gps_pose)
		gps_pose = new particle_filter::Pose(); 

	*gps_pose = gps_reading;

	//Gives reading directly to filter to attempt to calibrate if necessary. 
	if (!pf->calibrated()) {

		//Checks if odometry reading is available before proceeding. 
		if (odom) {
			//Converts current odometry reading to Pose struct of particle filter class. 
			Pose odom_reading_pose = Pose();
			odom_reading_pose.x = odom->pose.pose.position.x; 
			odom_reading_pose.y = odom->pose.pose.position.y; 

			//Gets theta. 
			double x = odom->pose.pose.orientation.x; 
			double y = odom->pose.pose.orientation.y;
			double z = odom->pose.pose.orientation.z;
			double w = odom->pose.pose.orientation.w; 
			odom_reading_pose.theta = get_yaw_from_quaternion(Eigen::Vector4d(x, y, z, w)); 

			//Converts gps reading to Pose struct. 
			Pose gps_reading_pose = Pose();
			gps_reading_pose.x = gps_reading.x; 
			gps_reading_pose.y = gps_reading.y; 
	
			pf->add_readings_for_calibration(&gps_reading_pose, &odom_reading_pose);
		}
	}
}

void imu_callback(sensor_msgs::Imu imu_reading) {
	if (!imu)
		imu = new sensor_msgs::Imu();

	*imu = imu_reading;

	//Gets quaternion components from reading. 
	double x = imu->orientation.x; 
	double y = imu->orientation.y; 
	double z = imu->orientation.z; 
	double w = imu->orientation.w;

	//Converts them to yaw. 
	imu_yaw = get_yaw_from_quaternion(Eigen::Vector4d(x, y, z, w));
}

void odom_callback(nav_msgs::Odometry odom_reading) {
	if (!odom)
		odom = new nav_msgs::Odometry(); 

	*odom = odom_reading;
}

void weigh_using_gps_and_imu(Particle *particle, void **args) {
	double gps_weight; 
	double imu_weight; 
	
	//Calculates gps component of weight. 
	if (0)
		gps_weight = 1.0; 
	else {
		double gps_x_variance = 2.172; 
		double gps_y_variance = 7.721; 

		double x_weight = 1 / (gps_x_variance + exp(std::abs(particle->pose.x - gps_pose->x))); 
		double y_weight = 1 / (gps_y_variance + exp(std::abs(particle->pose.y - gps_pose->y)));

		//Weighs particle using experimentally determined loss function. 
		gps_weight = x_weight * y_weight; 
	}

	//Calculates IMU component of weight. 
	if (0)
		imu_weight = 1.0;
	else {
		//Weighs particle using imu. 
		imu_weight = 1 / (1 + exp(std::abs(particle->pose.theta - imu_yaw)));
	}

	//Assigns mixture of imu and gps weights to particle.  particle->weight =  imu_weight; //gps_weight + imu_weight; 
	particle->weight = 5 * gps_weight;//imu_weight * gps_weight;
}

particle_filter::Particle_vector read_in_particles(ParticleFilter *pf, ros::Publisher *estimate_pub) {
	std::vector<Particle> particles = pf->get_particles(); 

	//Creates particle vector with necessary number of particles. 
	particle_filter::Particle_vector particle_vector; 
	particle_vector.particles = std::vector<particle_filter::Particle>(particles.size());


	//Copies each particle onto message. 
	for (int i = 0; i < particles.size(); i++) {
		//Copies pose. 
		particle_vector.particles[i].pose.x = particles[i].pose.x; 
		particle_vector.particles[i].pose.y = particles[i].pose.y; 
		particle_vector.particles[i].pose.theta = particles[i].pose.theta;

		//Copies weight. 
		particle_vector.particles[i].weight = particles[i].weight; 
	}

	//Gets pose estimate for robot. 
	Pose pose_estimate = pf->get_pose_estimate();
	particle_filter::Pose std_pose_estimate = particle_filter::Pose(); 

	std_pose_estimate.x = particle_vector.pose_estimate.x = pose_estimate.x;
	std_pose_estimate.y = particle_vector.pose_estimate.y = pose_estimate.y;
	std_pose_estimate.theta = particle_vector.pose_estimate.theta = pose_estimate.theta;

	//Publishes pose estimate standard Pose message. 
	estimate_pub->publish(std_pose_estimate); 

	return particle_vector; 
}

Pose odom_to_pose_reading(nav_msgs::Odometry *odom_reading) {
	Pose reading;

	//Gets translational reading. 
	reading.x = odom_reading->pose.pose.position.x; 
	reading.y = odom_reading->pose.pose.position.y; 

	//Gets quaternion components from reading. 
	double x = odom_reading->pose.pose.orientation.x;
	double y = odom_reading->pose.pose.orientation.y; 
	double z = odom_reading->pose.pose.orientation.z; 
	double w = odom_reading->pose.pose.orientation.w; 

	//Gets rotational reading. 
	reading.theta = get_yaw_from_quaternion(Eigen::Vector4d(x, y, z, w)); 

	return reading; 
}

int main(int argc, char **argv) {
	//Initialize particle filter node. 
	ros::init(argc, argv, "particle_filter"); 
	ros::NodeHandle node; 

	//Initialize publisher and rate. 
	ros::Publisher pub = node.advertise<particle_filter::Particle_vector>("particle_filter", 1000); 
	ros::Rate loop_rate(10); 

	ros::Publisher estimate_pub = node.advertise<particle_filter::Pose>("jackal/pose_estimate", 1000); 

	//Initialises the particle filter. 
	pf = new ParticleFilter(500, &weigh_using_gps_and_imu); 

	//Subscribes to Odometry topic. 
	ros::Subscriber odom_sub = node.subscribe("jackal_velocity_controller/odom", 1000, odom_callback); 

	//Subscribes to GPS topic. 
	ros::Subscriber gps_sub = node.subscribe("jackal/gps_estimate", 1000, gps_callback); 

	//Subscribes to IMU topic. 
	//ros::Subscriber imu_sub = node.subscribe("imu/data", 1000, imu_callback); 

	//Waits for first odometry reading. 
	while (!can_start() && ros::ok())
		ros::spinOnce();

	while (ros::ok()) {
		//Publishes current set of particles and sleeps.
		pub.publish(read_in_particles(pf, &estimate_pub)); 

		//Elapses time for the filter. 
		Pose reading = odom_to_pose_reading(odom);
		pf->elapse_time(&reading);

		//Weighs particles based on sensor readings. //TODO add sensor readings. 
		pf->weigh_particles(0);

		//Now resamples them. 
		pf->resample_particles();

		//Reads new messages and sleeps. 
		ros::spinOnce(); 
		loop_rate.sleep(); 
	}

	return 0; 
}

