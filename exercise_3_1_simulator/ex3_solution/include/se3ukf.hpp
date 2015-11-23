// This source code is intended for use in the teaching course "Vision-Based Navigation" in summer term 2015 at Technical University Munich only.
// Copyright 2015 Vladyslav Usenko, Joerg Stueckler, Technical University Munich


#ifndef SE3UKF_HPP_
#define SE3UKF_HPP_

#include <sophus/se3.hpp>
#include <iostream>
#include <list>
#include <boost/thread/mutex.hpp>

template<typename _Scalar>
class SE3UKF {

private:

	static const int STATE_SIZE = 15;
	static const int NUM_SIGMA_POINTS = 2 * STATE_SIZE + 1;

	typedef Sophus::SE3Group<_Scalar> SE3Type;
	typedef Sophus::SO3Group<_Scalar> SO3Type;

	typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;
	typedef Eigen::Matrix<_Scalar, 6, 1> Vector6;
	typedef Eigen::Matrix<_Scalar, 12, 1> Vector12;
	typedef Eigen::Matrix<_Scalar, 15, 1> Vector15;

	typedef Eigen::Matrix<_Scalar, 3, 3> Matrix3;
	typedef Eigen::Matrix<_Scalar, 6, 6> Matrix6;
	typedef Eigen::Matrix<_Scalar, 12, 12> Matrix12;
	typedef Eigen::Matrix<_Scalar, 15, 15> Matrix15;

	typedef Eigen::Matrix<_Scalar, 15, 3> Matrix15_3;
	typedef Eigen::Matrix<_Scalar, 15, 6> Matrix15_6;

	typedef std::vector<SE3Type, Eigen::aligned_allocator<SE3Type> > SE3TypeVector;
	typedef std::vector<Vector3, Eigen::aligned_allocator<Vector3> > Vector3Vector;

	SE3Type pose;
	Vector3 linear_velocity;
	Vector3 accel_bias;
	Vector3 gyro_bias;

	Matrix15 covariance;

	SE3TypeVector sigma_pose;
	Vector3Vector sigma_linear_velocity;
	Vector3Vector sigma_accel_bias;
	Vector3Vector sigma_gyro_bias;

	unsigned int iteration;

	// The function verifies that the angle increment is sufficiently small not to hit singularity.
	// If the error occures that means the filter has diverged.
	void print_if_too_large(const Vector3 & update, const std::string & name) {
		if (update.array().abs().maxCoeff() > M_PI / 4) {
			std::cout << "[" << iteration << "]" << name
					<< " has too large elements " << std::endl
					<< update.transpose() << std::endl;
			exit(-1);
		}

	}

	// Compute sigma points from covariance matrix
	void compute_sigma_points(const Vector15 & delta = Vector15::Zero()) {
		print_if_too_large(delta.template segment<3>(3), "Delta");

		Eigen::LLT<Matrix15> llt_of_covariance(covariance);
		assert(llt_of_covariance.info() == Eigen::Success);
		Matrix15 L = llt_of_covariance.matrixL();

		sigma_pose[0] = pose * SE3Type::exp(delta.template head<6>());
        sigma_linear_velocity[0] = linear_velocity + delta.template segment<3>(6);
		sigma_accel_bias[0] = accel_bias + delta.template segment<3>(9);
		sigma_gyro_bias[0] = gyro_bias + delta.template segment<3>(12);

		for (int i = 0; i < STATE_SIZE; i++) {
			Vector15 eps = L.col(i);

			sigma_pose[1 + i] = pose
					* SE3Type::exp(
							delta.template head<6>() + eps.template head<6>());

			sigma_linear_velocity[1 + i] = linear_velocity
					+ delta.template segment<3>(6) + eps.template segment<3>(6);

			sigma_accel_bias[1 + i] = accel_bias + delta.template segment<3>(9)
					+ eps.template segment<3>(9);

			sigma_gyro_bias[1 + i] = gyro_bias + delta.template segment<3>(12)
					+ eps.template segment<3>(12);

			sigma_pose[1 + STATE_SIZE + i] = pose
					* SE3Type::exp(
							delta.template head<6>() - eps.template head<6>());

			sigma_linear_velocity[1 + STATE_SIZE + i] = linear_velocity
					+ delta.template segment<3>(6) - eps.template segment<3>(6);

			sigma_accel_bias[1 + STATE_SIZE + i] = accel_bias
					+ delta.template segment<3>(9) - eps.template segment<3>(9);

			sigma_gyro_bias[1 + STATE_SIZE + i] = gyro_bias
					+ delta.template segment<3>(12)
					- eps.template segment<3>(12);

			print_if_too_large(eps.template segment<3>(3), "Epsilon");
			print_if_too_large(
					delta.template segment<3>(3) + eps.template segment<3>(3),
					"Delta + Epsilon");
			print_if_too_large(
					delta.template segment<3>(3) - eps.template segment<3>(3),
					"Delta - Epsilon");
		}
	}

	// Compute mean of the sigma points.
	// For SE3 an iterative method should be used.
    void compute_mean(  SE3Type & mean_pose, Vector3 & mean_linear_velocity,
                        Vector3 & mean_accel_bias, Vector3 & mean_gyro_bias) {

		mean_linear_velocity.setZero();
		mean_accel_bias.setZero();
		mean_gyro_bias.setZero();
		for (int i = 0; i < NUM_SIGMA_POINTS; i++) {
			mean_linear_velocity += sigma_linear_velocity[i];
			mean_accel_bias += sigma_accel_bias[i];
			mean_gyro_bias += sigma_gyro_bias[i];
		}
		mean_linear_velocity /= NUM_SIGMA_POINTS;
		mean_accel_bias /= NUM_SIGMA_POINTS;
		mean_gyro_bias /= NUM_SIGMA_POINTS;

		mean_pose = sigma_pose[0];
		Vector6 delta;

		const static int max_iterations = 1000;
		int iterations = 0;

		do {
			delta.setZero();

			for (int i = 0; i < NUM_SIGMA_POINTS; i++) {
				delta += SE3Type::log(mean_pose.inverse() * sigma_pose[i]);
			}
			delta /= NUM_SIGMA_POINTS;

			mean_pose *= SE3Type::exp(delta);
			iterations++;
        } while (   delta.array().abs().maxCoeff() > Sophus::SophusConstants<_Scalar>::epsilon()
                    && iterations < max_iterations);
	}


	// Compute mean and covariance from sigma points.
	// For computing mean previous function can be used.
    void compute_mean_and_covariance() {
		compute_mean(pose, linear_velocity, accel_bias, gyro_bias);
		covariance.setZero();

		for (int i = 0; i < NUM_SIGMA_POINTS; i++) {
			Vector15 eps;

			eps.template head<6>() = SE3Type::log(pose.inverse() * sigma_pose[i]);
            eps.template segment<3>(6) = sigma_linear_velocity[i] - linear_velocity;
			eps.template segment<3>(9) = sigma_accel_bias[i] - accel_bias;
			eps.template segment<3>(12) = sigma_gyro_bias[i] - gyro_bias;

			covariance += eps * eps.transpose();
		}
		covariance /= 2;
	}


public:
	typedef boost::shared_ptr<SE3UKF> Ptr;

	SE3UKF(const SE3Type & initial_pose,
			const Vector3 & initial_linear_velocity,
			const Vector3 & initial_accel_bias,
			const Vector3 & initial_gyro_bias,
			const Matrix15 & initial_covariance) {

		sigma_pose.resize(NUM_SIGMA_POINTS);
		sigma_linear_velocity.resize(NUM_SIGMA_POINTS);
		sigma_accel_bias.resize(NUM_SIGMA_POINTS);
		sigma_gyro_bias.resize(NUM_SIGMA_POINTS);

		iteration = 0;
		pose = initial_pose;
		linear_velocity = initial_linear_velocity;
		accel_bias = initial_accel_bias;
		gyro_bias = initial_gyro_bias;
		covariance = initial_covariance;

	}

	SE3UKF(const SE3UKF & other) {

		sigma_pose.resize(NUM_SIGMA_POINTS);
		sigma_linear_velocity.resize(NUM_SIGMA_POINTS);
		sigma_accel_bias.resize(NUM_SIGMA_POINTS);
		sigma_gyro_bias.resize(NUM_SIGMA_POINTS);

		iteration = other.iteration;
		pose = other.pose;
		linear_velocity = other.linear_velocity;
		accel_bias = other.accel_bias;
		gyro_bias = other.gyro_bias;
		covariance = other.covariance;

	}

	SE3UKF & operator=(const SE3UKF & other) {

		sigma_pose.resize(NUM_SIGMA_POINTS);
		sigma_linear_velocity.resize(NUM_SIGMA_POINTS);
		sigma_accel_bias.resize(NUM_SIGMA_POINTS);
		sigma_gyro_bias.resize(NUM_SIGMA_POINTS);

		iteration = other.iteration;
		pose = other.pose;
		linear_velocity = other.linear_velocity;
		accel_bias = other.accel_bias;
		gyro_bias = other.gyro_bias;
		covariance = other.covariance;
		return *this;
	}


    // TODO: exercise 2 b)
	// Predict new state and covariance with IMU measurement.
    void predict(   const Vector3 & accel_measurement,
                    const Vector3 & gyro_measurement, const _Scalar dt,
                    const Matrix3 & accel_noise, const Matrix3 & gyro_noise) {

        // 1. compute sigma points of current state
        compute_sigma_points(covariance);

        // 2. for each sigma point, apply given IMU motion model
        for(int i=0; i<NUM_SIGMA_POINTS; i++) {
            // get rotation and translation from the pose
//            Matrix3 rot = sigma_pose[i].topLeftCorner(3,3);
//            Vector3 t = sigma_pose[i].topRightCorner(3,1);

            sigma_pose[i].topRightCorner(3,1)   = sigma_pose[i].topRightCorner(3,1) + sigma_linear_velocity[i] * dt;
            sigma_linear_velocity[i]            = sigma_linear_velocity[i] + (sigma_pose[i].topLeftCorner(3,3) * (accel_measurement - sigma_accel_bias[i]) - 9.81f) * dt;
            sigma_pose[i].topLeftCorner(3,3)    = sigma_pose[i].topLeftCorner(3,3) * exp((gyro_measurement-sigma_gyro_bias[i])*dt);
        }

        // 3. compute new mean and covarience.
        compute_mean_and_covariance();
	}


    // TODO: exercise 2 c)
	// Apply 6d pose measurement.
	void measurePose(const SE3Type & measured_pose,
			const Matrix6 & measurement_noise) {

	}

	SE3Type get_pose() const {
		return pose;
	}

	Vector3 get_linear_velocity() const {
		return linear_velocity;
	}

	Matrix15 get_covariance() const {
		return covariance;
	}

	Vector3 get_accel_bias() const {
		return accel_bias;
	}

	Vector3 get_gyro_bias() const {
		return gyro_bias;
	}

	// The function is a very basic test to check sigma points mean and covariance computation.
	// It computes sigma points from the current distribution and then computes the distribution from the sigma points.
	// In the correct implementation two distributions should be identical up to numeric errors.
	void test_sigma_points() {
		SE3Type pose1 = pose;
		Vector3 linear_velocity1 = linear_velocity;
		Vector3 accel_bias1 = accel_bias;
		Vector3 gyro_bias1 = gyro_bias;
		Matrix15 covariance1 = covariance;

		compute_sigma_points();
		compute_mean_and_covariance();

		SE3Type pose2 = pose;
		Vector3 linear_velocity2 = linear_velocity;
		Vector3 accel_bias2 = accel_bias;
		Vector3 gyro_bias2 = gyro_bias;
		Matrix15 covariance2 = covariance;

		if ((covariance1 - covariance2).array().abs().maxCoeff() > 1e-5) {
			std::cerr << "[" << iteration << "]" << "Covariance miscalculated\n"
					<< covariance1 << "\n\n" << covariance2 << "\n\n";
			assert(false);
		}

		if ((linear_velocity1 - linear_velocity2).array().abs().maxCoeff()
				> 1e-5) {
			std::cerr << "[" << iteration << "]" << "Velocity miscalculated\n"
					<< linear_velocity1 << "\n\n" << linear_velocity2 << "\n\n";
			assert(false);
		}

		if ((accel_bias1 - accel_bias2).array().abs().maxCoeff() > 1e-5) {
			std::cerr << "[" << iteration << "]"
					<< "Accelerometer bias miscalculated\n" << accel_bias1
					<< "\n\n" << accel_bias2 << "\n\n";
			assert(false);
		}

		if ((gyro_bias1 - gyro_bias2).array().abs().maxCoeff() > 1e-5) {
			std::cerr << "[" << iteration << "]"
					<< "Gyroscope bias miscalculated\n" << gyro_bias1 << "\n\n"
					<< gyro_bias2 << "\n\n";
			assert(false);
		}

	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif /* SE3UKF_HPP_ */
