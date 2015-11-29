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

    // mean
    SE3Type pose_;
    Vector3 linear_velocity_;
    Vector3 accel_bias_;
    Vector3 gyro_bias_;

    // covariance
    Matrix15 covariance_;

    // sigma points
    SE3TypeVector sigma_pose_;
    Vector3Vector sigma_linear_velocity_;
    Vector3Vector sigma_accel_bias_;
    Vector3Vector sigma_gyro_bias_;

    unsigned int iteration_;

    // The function verifies that the angle increment is sufficiently small not to hit singularity.
    // If the error occures that means the filter has diverged.
    void print_if_too_large(const Vector3 & update, const std::string & name) {
        if (update.array().abs().maxCoeff() > M_PI / 4) {
            std::cout << "[" << iteration_ << "]" << name
                      << " has too large elements " << std::endl
                      << update.transpose() << std::endl;
            exit(-1);
        }

    }

    /**
     * Compute sigma points from covariance matrix (line 52 and 53 in http://arxiv.org/pdf/1107.1119.pdf p.12)
     * with g(u_t,sigma_old) = sigma_old + u_t // u_t = delta ?
     * @param delta _delta_ of measurement, odometry and bias // local vector-space view ?
     *   delta.template head<6>: measurement in SE3
     *   delta.template segment<3>(6): linear velocity
     *   delta.template segment<3>(9/12): gyro_bias_ and gyro_bias_
     *   (it is probably only needed for line 64 in http://arxiv.org/pdf/1107.1119.pdf p.12)
     * @param covariance_
     * @param pose_
     * @param linear_velocity
     * @param accel_bias_
     * @param gyro_bias_
     *
     * @return sigma_pose_
     * @return sigma_linear_velocity_
     * @return sigma_accel_bias_
     * @return sigma_gyro_bias_
    */
    void compute_sigma_points(const Vector15 & delta = Vector15::Zero()) {
        print_if_too_large(delta.template segment<3>(3), "Delta");

        // computes Cholesky decomposition, result is a lower left triangle (llt) matrix L
        // with L * L^T = original matrix
        Eigen::LLT<Matrix15> llt_of_covariance(covariance_);
        assert(llt_of_covariance.info() == Eigen::Success);
        Matrix15 L = llt_of_covariance.matrixL();

        // sigma(0)_old = mean
        // sigma(0)_new = sigma(0)_old + delta ( equals g(u_t, sigma_old) in line 53)
        sigma_pose_[0] = pose_ * SE3Type::exp(delta.template head<6>());
        sigma_linear_velocity_[0] = linear_velocity_ + delta.template segment<3>(6);
        sigma_accel_bias_[0] = accel_bias_ + delta.template segment<3>(9);
        sigma_gyro_bias_[0] = gyro_bias_ + delta.template segment<3>(12);

        for (int i = 0; i < STATE_SIZE; i++) {
            Vector15 eps = L.col(i);

            // sigma[1-15]_new = mean_old + sqrt(covariance_)[i] + delta // sqrt(covariance_)[i] = L.col(i)
            sigma_pose_[1 + i] = pose_
                                 * SE3Type::exp(
                                     delta.template head<6>() + eps.template head<6>());

            sigma_linear_velocity_[1 + i] = linear_velocity_
                                            + delta.template segment<3>(6) + eps.template segment<3>(6);

            sigma_accel_bias_[1 + i] = accel_bias_ + delta.template segment<3>(9)
                                       + eps.template segment<3>(9);

            sigma_gyro_bias_[1 + i] = gyro_bias_ + delta.template segment<3>(12)
                                      + eps.template segment<3>(12);

            // sigma[16-30]_new = mean_old - sqrt(covariance_)[i] + delta // sqrt(covariance_)[i] = L.col(i)
            sigma_pose_[1 + STATE_SIZE + i] = pose_
                                              * SE3Type::exp(
                                                  delta.template head<6>() - eps.template head<6>());

            sigma_linear_velocity_[1 + STATE_SIZE + i] = linear_velocity_
                    + delta.template segment<3>(6) - eps.template segment<3>(6);

            sigma_accel_bias_[1 + STATE_SIZE + i] = accel_bias_
                                                    + delta.template segment<3>(9) - eps.template segment<3>(9);

            sigma_gyro_bias_[1 + STATE_SIZE + i] = gyro_bias_
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

    /**
     * Compute mean of the sigma points. (line 54 in http://arxiv.org/pdf/1107.1119.pdf p.12)
     * details not important ?
     * For SE3 an iterative method should be used.
     * @param sigma_pose_
     * @param sigma_linear_velocity_
     * @param sigma_accel_bias_
     * @param sigma_gyro_bias_
     *
     * @return mean_pose
     * @return mean_linear_velocity
     * @return mean_accel_bias
     * @return mean_gyro_bias
     */
    void compute_mean(  SE3Type & mean_pose, Vector3 & mean_linear_velocity,
                        Vector3 & mean_accel_bias, Vector3 & mean_gyro_bias) {

        mean_linear_velocity.setZero();
        mean_accel_bias.setZero();
        mean_gyro_bias.setZero();
        for (int i = 0; i < NUM_SIGMA_POINTS; i++) {
            mean_linear_velocity += sigma_linear_velocity_[i];
            mean_accel_bias += sigma_accel_bias_[i];
            mean_gyro_bias += sigma_gyro_bias_[i];
        }
        mean_linear_velocity /= NUM_SIGMA_POINTS;
        mean_accel_bias /= NUM_SIGMA_POINTS;
        mean_gyro_bias /= NUM_SIGMA_POINTS;

        mean_pose = sigma_pose_[0];
        Vector6 delta;

        const static int max_iterations = 1000;
        int iterations = 0;

        do {
            delta.setZero();

            for (int i = 0; i < NUM_SIGMA_POINTS; i++) {
                delta += SE3Type::log(mean_pose.inverse() * sigma_pose_[i]);
            }
            delta /= NUM_SIGMA_POINTS;

            mean_pose *= SE3Type::exp(delta);
            iterations++;
        } while (   delta.array().abs().maxCoeff() > Sophus::SophusConstants<_Scalar>::epsilon()
                    && iterations < max_iterations);
    }


    /**
     * Compute mean and covariance from sigma points. (line 54 and 55 in http://arxiv.org/pdf/1107.1119.pdf p.12)
     * straightforward sum of sigma points
     * For computing mean, previous function can be used.
     * @param sigma_pose_
     * @param sigma_linear_velocity_
     * @param sigma_accel_bias_
     * @param sigma_gyro_bias_
     *
     * @return pose_
     * @return linear_velocity_
     * @return accel_bias_
     * @return gyro_bias_
     *
     * @return covariance_
     */
    void compute_mean_and_covariance() {
        compute_mean(pose_, linear_velocity_, accel_bias_, gyro_bias_);
        covariance_.setZero();

        for (int i = 0; i < NUM_SIGMA_POINTS; i++) {
            Vector15 eps;

            eps.template head<6>() = SE3Type::log(pose_.inverse() * sigma_pose_[i]);
            eps.template segment<3>(6) = sigma_linear_velocity_[i] - linear_velocity_;
            eps.template segment<3>(9) = sigma_accel_bias_[i] - accel_bias_;
            eps.template segment<3>(12) = sigma_gyro_bias_[i] - gyro_bias_;

            covariance_ += eps * eps.transpose();
        }
        covariance_ /= 2;
    }


public:
    typedef boost::shared_ptr<SE3UKF> Ptr;

    SE3UKF(const SE3Type & initial_pose,
           const Vector3 & initial_linear_velocity,
           const Vector3 & initial_accel_bias,
           const Vector3 & initial_gyro_bias,
           const Matrix15 & initial_covariance) {

        sigma_pose_.resize(NUM_SIGMA_POINTS);
        sigma_linear_velocity_.resize(NUM_SIGMA_POINTS);
        sigma_accel_bias_.resize(NUM_SIGMA_POINTS);
        sigma_gyro_bias_.resize(NUM_SIGMA_POINTS);

        iteration_ = 0;
        pose_ = initial_pose;
        linear_velocity_ = initial_linear_velocity;
        accel_bias_ = initial_accel_bias;
        gyro_bias_ = initial_gyro_bias;
        covariance_ = initial_covariance;

    }

    SE3UKF(const SE3UKF & other) {

        sigma_pose_.resize(NUM_SIGMA_POINTS);
        sigma_linear_velocity_.resize(NUM_SIGMA_POINTS);
        sigma_accel_bias_.resize(NUM_SIGMA_POINTS);
        sigma_gyro_bias_.resize(NUM_SIGMA_POINTS);

        iteration_ = other.iteration_;
        pose_ = other.pose_;
        linear_velocity_ = other.linear_velocity_;
        accel_bias_ = other.accel_bias_;
        gyro_bias_ = other.gyro_bias_;
        covariance_ = other.covariance_;

    }

    SE3UKF & operator=(const SE3UKF & other) {

        sigma_pose_.resize(NUM_SIGMA_POINTS);
        sigma_linear_velocity_.resize(NUM_SIGMA_POINTS);
        sigma_accel_bias_.resize(NUM_SIGMA_POINTS);
        sigma_gyro_bias_.resize(NUM_SIGMA_POINTS);

        iteration_ = other.iteration_;
        pose_ = other.pose_;
        linear_velocity_ = other.linear_velocity_;
        accel_bias_ = other.accel_bias_;
        gyro_bias_ = other.gyro_bias_;
        covariance_ = other.covariance_;
        return *this;
    }


    // TODO: exercise 2 b)
    /**
     * Predict new state and covariance with IMU measurement.
     * @param accel_measurement
     * @param gyro_measurement
     * @param dt
     * @param accel_noice
     * @param gyro_noice
     *
     * @param "old mean and covariance"
     *
     * @return mean_pose
     * @return mean_linear_velocity
     * @return mean_accel_bias
     * @return mean_gyro_bias
     * 	==> new mean
     * @return covariance_ new covariance
     */
    void predict(   const Vector3 & accel_measurement,
                    const Vector3 & gyro_measurement,
                    const _Scalar dt,
                    const Matrix3 & accel_noise,
                    const Matrix3 & gyro_noise) {

        // TODO: add accel_noise and gyro_noise

        // 1. compute sigma points of current state
        // sigmaPoints_(t-1) = (mean_(t-1), mean_(t-1) + sqrt(covariance_(t-1), mean_(t-1) - sqrt(covariance_(t-1))) p.12 l.52
        compute_sigma_points(); // DELETE: add delta? addendum: probably not, because the delta is also in line 64

        // 2. for each sigma point, apply given IMU motion model
        for(int i=0; i<NUM_SIGMA_POINTS; i++) {
            // get rotation and translation from the pose
            // Matrix3 rot = sigma_pose_[i].topLeftCorner(3,3);
            // Vector3 t = sigma_pose_[i].topRightCorner(3,1);

            // p.15 in http://arxiv.org/pdf/1107.1119.pdf g(u, XI)
            sigma_pose_[i].translation() = sigma_pose_[i].translation() + ( sigma_linear_velocity_[i] * dt );
            sigma_linear_velocity_[i]    = sigma_linear_velocity_[i] +
                                           ((sigma_pose_[i].rotationMatrix() * (accel_measurement - sigma_accel_bias_[i]))
                                            - Vector3(0,0,-9.8)) * dt;
            sigma_pose_[i].setRotationMatrix( sigma_pose_[i].rotationMatrix() *
                                              Sophus::SO3Group<_Scalar>::exp( (gyro_measurement-sigma_gyro_bias_[i])*dt ).matrix());
        }

        // 3. compute new mean and covarience.
        // p.12 l.54 + 55
        compute_mean_and_covariance();
	// TODO: add noise
    }


    // TODO: exercise 2 c)  (equals correction step in http://arxiv.org/pdf/1107.1119.pdf p.12)
    /**
     * Apply 6d pose measurement.
     * @param measured_pose z_t
     * @param measurement_noise Q_t
     *
     * @return mean_pose
     * @return mean_linear_velocity
     * @return mean_accel_bias
     * @return mean_gyro_bias
     * 	==> corrected mean
     *
     * @return covariance_ corrected covariance
     */
    void measurePose(   const SE3Type & measured_pose,
                        const Matrix6 & measurement_noise) {

        // 58 and 59
        // This overwrites our old mean and covariance, therefore we saved it before
        compute_mean_and_covariance();
        // We are only  interested in the pose
        Matrix6 S = covariance_.template block<6,6>(0,0) + measurement_noise;
	
	// 60  K = K_xz // only if mean = zhat
	Matrix15_6 K = covariance_.template block<15,6>(0,0);
	// 62 K = K_t
        K = K * S.inverse();
        // auxMean = K * (Vector15(measured_pose, measurement_noise) - z);
	Vector15 delta = K * SE3Type::log(pose_.inverse() * measured_pose);
	// 63
	covariance_ = covariance_ - K * S * K.transpose();
	
	// Symmetrize covariance
	covariance_ = (covariance_ + covariance_.transpose()) / 2;

	// 64 - 66
        compute_sigma_points(delta); // only if mean = zhat
        compute_mean_and_covariance();
    }

    SE3Type get_pose() const {
        return pose_;
    }

    Vector3 get_linear_velocity() const {
        return linear_velocity_;
    }

    Matrix15 get_covariance() const {
        return covariance_;
    }

    Vector3 get_accel_bias() const {
        return accel_bias_;
    }

    Vector3 get_gyro_bias() const {
        return gyro_bias_;
    }

    // The function is a very basic test to check sigma points mean and covariance_ computation.
    // It computes sigma points from the current distribution and then computes the distribution from the sigma points.
    // In the correct implementation two distributions should be identical up to numeric errors.
    void test_sigma_points() {
        SE3Type pose1 = pose_;
        Vector3 linear_velocity1 = linear_velocity_;
        Vector3 accel_bias1 = accel_bias_;
        Vector3 gyro_bias1 = gyro_bias_;
        Matrix15 covariance1 = covariance_;

        compute_sigma_points();
        compute_mean_and_covariance();

        SE3Type pose2 = pose_;
        Vector3 linear_velocity2 = linear_velocity_;
        Vector3 accel_bias2 = accel_bias_;
        Vector3 gyro_bias2 = gyro_bias_;
        Matrix15 covariance2 = covariance_;

        if ((covariance1 - covariance2).array().abs().maxCoeff() > 1e-5) {
            std::cerr << "[" << iteration_ << "]" << "covariance_ miscalculated\n"
                      << covariance1 << "\n\n" << covariance2 << "\n\n";
            assert(false);
        }

        if ((linear_velocity1 - linear_velocity2).array().abs().maxCoeff()
                > 1e-5) {
            std::cerr << "[" << iteration_ << "]" << "Velocity miscalculated\n"
                      << linear_velocity1 << "\n\n" << linear_velocity2 << "\n\n";
            assert(false);
        }

        if ((accel_bias1 - accel_bias2).array().abs().maxCoeff() > 1e-5) {
            std::cerr << "[" << iteration_ << "]"
                      << "Accelerometer bias miscalculated\n" << accel_bias1
                      << "\n\n" << accel_bias2 << "\n\n";
            assert(false);
        }

        if ((gyro_bias1 - gyro_bias2).array().abs().maxCoeff() > 1e-5) {
            std::cerr << "[" << iteration_ << "]"
                      << "Gyroscope bias miscalculated\n" << gyro_bias1 << "\n\n"
                      << gyro_bias2 << "\n\n";
            assert(false);
        }

    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif /* SE3UKF_HPP_ */



