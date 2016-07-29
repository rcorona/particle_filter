#include <particle_filter/particle_filter.h>
#include <iostream> 
#include <random>
#include <math.h>
#include <algorithm> 

ParticleFilter::ParticleFilter(int num_particles, void (*weighing_func)(Particle *, void **)) {
	//Resizes particle vector. 
	this->num_particles = num_particles; 
	particles.resize(num_particles); 

	//Sets weighing function to given one. 
	weighing_function = weighing_func; 

	//Mean actual movement in error model experiments. 
	double mean_movement = 1.88; 

	//Set x and y error.
	//TODO CURRENTLY SET TO CONSTRAINED MOTION MODEL. 
	x_error.mean_proportion = -0.00; //-0.12966300393266664 / mean_movement;
	x_error.variance_proportion = 0.03; //0.032975724500232106 / mean_movement; 

	y_error.mean_proportion = 0.00; //0.7257227026205334 / mean_movement;
	y_error.variance_proportion = 0.07; //0.07870249376260666 / mean_movement;

	//Set rotation error. 
	rotation_error.variance_proportion = 0.1; //0.0011277530205 / mean_movement;
	rotation_error.mean_proportion = .01 / mean_movement;

	//Sets odometry reading to null. 
	odom = 0;

	//Initially heading is not calibrated. 
	calibrated_heading = false;

	calibration_values.odom_start = calibration_values.gps_start = 0; 
	calibration_values.odom_buff = calibration_values.gps_buff = 0; 
	calibration_values.odom_end = calibration_values.gps_end = 0; 
}

ParticleFilter::~ParticleFilter() {
	//Frees odometry reading memory if still allocated. 
	if (odom)
		delete odom; 
}

bool ParticleFilter::calibrated() {
	return calibrated_heading; 
}

void ParticleFilter::add_readings_for_calibration(Pose *gps_reading, Pose *odom_reading) {
	//If first calibration value, then saves it. 
	if (!calibration_values.odom_start || !calibration_values.gps_start) {
		calibration_values.odom_start = new Pose(*odom_reading);
		calibration_values.gps_start = new Pose(*gps_reading); 
	}
	else {
		//If second value not set, see if it's far enough away to mitigate possibility of error. 
		double dx = gps_reading->x - calibration_values.gps_start->x;  
		double dy = gps_reading->y - calibration_values.gps_start->y; 

		//Creates new buffer if needed.
		if (!calibration_values.odom_buff || !calibration_values.gps_buff) {
			calibration_values.odom_buff = new Pose(*odom_reading); 
			calibration_values.gps_buff = new Pose(*gps_reading); 
		}
		//Checks if we may calibrates heading using first and second readings.
		else if (sqrt(pow(dx, 2) + pow(dy, 2)) > 0.5) {
			//Sets second point values and uses collected poses for calibration. 
			calibration_values.odom_end = new Pose(*odom_reading); 
			calibration_values.gps_end = new Pose(*gps_reading); 

			calibrate(); 
		}
		else { //Update buffer value. 
			*calibration_values.odom_buff = *odom_reading; 
			*calibration_values.gps_buff = *gps_reading; 
		}
	}
}

void ParticleFilter::calibrate() {	
	//Determines change in x & y for gps coordinates. 
	double gps_dx = calibration_values.gps_end->x - calibration_values.gps_buff->x; 
	double gps_dy = calibration_values.gps_end->y - calibration_values.gps_buff->y; 

	//Determines gps estimate of angle. 
	double gps_theta = atan2(gps_dy, gps_dx);

	//Determines change in x & y for odometry coordinates. 
	double odom_dx = calibration_values.odom_end->x - calibration_values.odom_buff->x; 
	double odom_dy = calibration_values.odom_end->y - calibration_values.odom_buff->y; 

	//Determines odometry's angle of translation. 
	double odom_trans_theta = atan2(odom_dy, odom_dx); 

	//Estimates necessary rotation to convert between odometry and gps translations. 
	double rot_angle = gps_theta - odom_trans_theta;

	//Creates rotation matrix based on necessary rotation angles.
	calibration_values.rot_matrix << cos(rot_angle), -sin(rot_angle), sin(rot_angle), cos(rot_angle); 

	//Sets initial position of particles based on calibrations. 
	for (int i = 0; i < num_particles; i++) {
		particles[i].pose.x = calibration_values.gps_end->x;
		particles[i].pose.y = calibration_values.gps_end->y; 
		particles[i].pose.theta = gps_theta;
	}

	//Sets calibrated flag so that pose estimate may now be used. 
	calibrated_heading = true; 
}

int ParticleFilter::get_num_particles() {
	return num_particles; 
}

Pose ParticleFilter::get_pose_estimate() {
	return pose_estimate; 
}

std::vector<Particle> ParticleFilter::get_particles() {
	return particles; 
}

Pose ParticleFilter::get_odom_diff(Pose *odom_reading) {
	//Will contain difference in readings. 
	Pose diff; 

	//Computes differences. 
	diff.x = odom_reading->x - odom->x; 
	diff.y = odom_reading->y - odom->y;
	diff.theta = odom_reading->theta - odom->theta; 

	//Remembers reading for next iteration. 
	odom->x = odom_reading->x; 
	odom->y = odom_reading->y;
	odom->theta = odom_reading->theta; 

	return diff; 
}

void ParticleFilter::elapse_time(Pose *odom_reading) {
	//Initializes current reading if needed. 
	if (!odom) {
		odom = new Pose();

		//Copies values. 
		odom->x = odom_reading->x; 
		odom->y = odom_reading->y;
		odom->theta = odom_reading->theta; 
	}

	//Gets reported difference in odometry since last reading. 
	Pose odom_diff = get_odom_diff(odom_reading);

	//Compute translation gaussian based on reading. 
	compute_translation_gaussians(&odom_diff); 

	//Elapses time for each particle given the reading.
	for (int i = 0; i < num_particles; i++)
		elapse_particle_time(&particles[i], &odom_diff);

	//Estimates current position. 
	estimate_current_position(); 
}

void ParticleFilter::elapse_particle_time(Particle *particle, Pose *reading) {
	//Samples errors. 
	double x_error = x_gauss(generator); 
	double y_error = y_gauss(generator); 
	double rotation_error = rotation_gauss(generator); 

	//Vectorizes translation reading. 
	Eigen::Vector2d reading_v(reading->x, reading->y);
	
	//Gets translation vector in gps using rotation matrix. 
	Eigen::Vector2d converted_reading = calibration_values.rot_matrix * reading_v; 

	//TODO remove, for testing. 
	double motion_angle = atan2(converted_reading(1), converted_reading(0)); 

	//Computes rotation matrix for particle. 
	
	//Fully constrained motion model. 
	double rot_angle = particle->pose.theta - motion_angle; 
	
	//SLIGHTLY CONSTRAINED double rot_angle = particle->pose.theta - pose_estimate.theta;  

	Eigen::Matrix2d rot_matrix;
	rot_matrix << cos(rot_angle), -sin(rot_angle), sin(rot_angle), cos(rot_angle); 

	//Rotates translation vector to bias it with particle heading. 
	Eigen::Vector2d final_reading = rot_matrix * converted_reading; 

	//Gets new pose estimate. 
	double dx = final_reading(0); 
	double dy = final_reading(1); 

	particle->pose.x += dx - x_error;  
	particle->pose.y += dy - y_error; 

	//Updates orientation. 
	particle->pose.theta += reading->theta - rotation_error;
	particle->pose.theta = wrap_angle(particle->pose.theta); 
}

void ParticleFilter::weigh_particles(void **args) {
	//Weighs each individual particle. 
	for (int i = 0; i < num_particles; i++)
		weighing_function(&particles[i], args); 
}

void ParticleFilter::compute_translation_gaussians(Pose *reading) {
	//Gets estimated total translation. 
	double estimated_translation = sqrt(pow(reading->x, 2) + pow(reading->y, 2));

	//Computes means. 
	double x_mean = x_error.mean_proportion * estimated_translation;  
	double y_mean = y_error.mean_proportion * estimated_translation; 
	double rotation_mean = rotation_error.mean_proportion * estimated_translation; 

	//Computes variances based on forward error model.
	double x_variance = x_error.variance_proportion * estimated_translation;
	double y_variance = y_error.variance_proportion * estimated_translation; 
	double rotation_variance = rotation_error.variance_proportion * estimated_translation; 

	//Computes std deviations. 
	double x_std_dev = sqrt(x_variance);  
	double y_std_dev = sqrt(y_variance); 
	double rotation_std_dev = sqrt(rotation_variance);

	//Computes gaussians. 
	x_gauss = std::normal_distribution<double>(x_mean, x_std_dev); 
	y_gauss = std::normal_distribution<double>(y_mean, y_std_dev); 
	rotation_gauss = std::normal_distribution<double>(rotation_mean, rotation_std_dev); 
}

void ParticleFilter::resample_particles() {
	//Computes sum of weights in order to normalize. 
	double w_sum = 0.0;

	for (int i = 1; i < num_particles; i++)
		w_sum += particles[i].weight; 

	//Computes cumulative sum vector of normalized weights. 
	std::vector<double> c_sum(num_particles); 
	c_sum[0] = particles[0].weight / w_sum; 

	for (int i = 1; i < num_particles; i++)
		c_sum[i] = c_sum[i - 1] + particles[i].weight / w_sum;

	//Computes vector of random numbers and sorts them for resampling.  
	std::vector<double> samples = std::vector<double>(num_particles);
	std::uniform_real_distribution<double> distribution(0.0, 1.0); 

	for (int i = 0; i < num_particles; i++)
		samples[i] = distribution(generator);

	std::sort(samples.begin(), samples.end());

	//Determines which particle indeces were sampled. 
	int i = 0, j = 0; 
	std::vector<Particle> new_particles = std::vector<Particle>(num_particles); 

	while (i < num_particles) {
		//If true, then current particle's new state corresponds to the j'th particle's state. 
		if (samples[i] < c_sum[j]) {
			//Copies particle. 
			new_particles[i] = particles[j]; 
			i++; 
		}
		else
			j++; 
	}

	//Updates particles to reflect resampled values. 
	particles = new_particles; 
}

void ParticleFilter::estimate_current_position() {
	double x_sum = 0.0, y_sum = 0.0, theta_sum = 0.0; 

	//Sums particle pose values. 
	for (int i = 0; i < num_particles; i++) {
		x_sum += particles[i].pose.x;
		y_sum += particles[i].pose.y; 
		theta_sum += particles[i].pose.theta; 
	}

	//Averages them to estimate position. 
	pose_estimate.x = x_sum / static_cast<double>(num_particles);
	pose_estimate.y = y_sum / static_cast<double>(num_particles); 
	pose_estimate.theta = theta_sum / static_cast<double>(num_particles); 

	std::cout << "X" << pose_estimate.x << std::endl; 
	std::cout << "Y" << pose_estimate.y << std::endl;
	std::cout << "T" << pose_estimate.theta << std::endl; 
}

double ParticleFilter::wrap_angle(double angle) {
	double pi = 3.1415926535897;
	double remainder = fmod(angle + pi, 2 * pi);

	if (remainder < 0) 
		remainder += 2 * pi; 
	
	return remainder - pi; 
}
