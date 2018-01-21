#include <arc_utilities/simple_robot_models.hpp>
#include <fast_kinematic_simulator/simple_pid_controller.hpp>
#include <fast_kinematic_simulator/simple_uncertainty_models.hpp>

#ifndef TNUVA_ROBOT_MODELS_HPP
#define TNUVA_ROBOT_MODELS_HPP

namespace tnuva_robot_models
{
    template<typename Configuration, typename Generator>
    class TnuvaRobot
    {
    public:

        virtual const Configuration& ResetPosition(const Configuration& position) = 0;

        virtual void ResetControllers() = 0;

        virtual void ApplyControlInput(const Eigen::VectorXd& control_input) = 0;

        virtual void ApplyControlInput(const Eigen::VectorXd& control_input, Generator& rng) = 0;

        virtual Eigen::VectorXd GenerateControlAction(const simple_se2_robot_model::SimpleSE2Configuration& target, const double controller_interval) = 0;
    };

    template<typename Generator>
    class TnuvaSE2Robot : public simple_robot_models::PointSphereBasicSE2Robot, TnuvaRobot<simple_se2_robot_model::SimpleSE2Configuration, Generator>
    {
    protected:

        simple_pid_controller::SimplePIDController x_axis_controller_;
        simple_pid_controller::SimplePIDController y_axis_controller_;
        simple_pid_controller::SimplePIDController zr_axis_controller_;
        simple_uncertainty_models::TruncatedNormalUncertainSensor x_axis_sensor_;
        simple_uncertainty_models::TruncatedNormalUncertainSensor y_axis_sensor_;
        simple_uncertainty_models::TruncatedNormalUncertainSensor zr_axis_sensor_;
        simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator x_axis_actuator_;
        simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator y_axis_actuator_;
        simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator zr_axis_actuator_;

    public:

        struct SE2_ROBOT_CONFIG
        {
            double kp;
            double ki;
            double kd;
            double integral_clamp;
            double velocity_limit;
            double acceleration_limit;
            double max_sensor_noise;
            double max_actuator_proportional_noise;
            double max_actuator_minimum_noise;
            double r_kp;
            double r_ki;
            double r_kd;
            double r_integral_clamp;
            double r_velocity_limit;
            double r_acceleration_limit;
            double r_max_sensor_noise;
            double r_max_actuator_proportional_noise;
            double r_max_actuator_minimum_noise;

            SE2_ROBOT_CONFIG()
            {
                kp = 0.0;
                ki = 0.0;
                kd = 0.0;
                integral_clamp = 0.0;
                velocity_limit = 0.0;
                acceleration_limit = 0.0;
                max_sensor_noise = 0.0;
                max_actuator_proportional_noise = 0.0;
                max_actuator_minimum_noise = 0.0;
                r_kp = 0.0;
                r_ki = 0.0;
                r_kd = 0.0;
                r_integral_clamp = 0.0;
                r_velocity_limit = 0.0;
                r_acceleration_limit = 0.0;
                r_max_sensor_noise = 0.0;
                r_max_actuator_proportional_noise = 0.0;
                r_max_actuator_minimum_noise = 0.0;
            }

            SE2_ROBOT_CONFIG(const double in_kp, const double in_ki, const double in_kd, const double in_integral_clamp, const double in_velocity_limit, const double in_acceleration_limit, const double in_max_sensor_noise, const double in_max_actuator_proportional_noise, const double in_max_actuator_minimum_noise, const double in_r_kp, const double in_r_ki, const double in_r_kd, const double in_r_integral_clamp, const double in_r_velocity_limit, const double in_r_acceleration_limit, const double in_r_max_sensor_noise, const double in_r_max_actuator_proportional_noise, const double in_r_max_actuator_minimum_noise)
            {
                kp = in_kp;
                ki = in_ki;
                kd = in_kd;
                integral_clamp = in_integral_clamp;
                velocity_limit = in_velocity_limit;
                acceleration_limit = in_acceleration_limit;
                max_sensor_noise = in_max_sensor_noise;
                max_actuator_proportional_noise = in_max_actuator_proportional_noise;
                max_actuator_minimum_noise = in_max_actuator_minimum_noise;
                r_kp = in_r_kp;
                r_ki = in_r_ki;
                r_kd = in_r_kd;
                r_integral_clamp = in_r_integral_clamp;
                r_velocity_limit = in_r_velocity_limit;
                r_acceleration_limit = in_r_acceleration_limit;
                r_max_sensor_noise = in_r_max_sensor_noise;
                r_max_actuator_proportional_noise = in_r_max_actuator_proportional_noise;
                r_max_actuator_minimum_noise = in_r_max_actuator_minimum_noise;
            }
        };

        TnuvaSE2Robot(const simple_se2_robot_model::SimpleSE2Configuration& initial_position,
                      const double position_distance_weight,
                      const double rotation_distance_weight,
                      const std::string& link_name,
                      const simple_robot_models::PointSphereGeometry& geometry,
                      const TnuvaSE2Robot::SE2_ROBOT_CONFIG& robot_config)
            : simple_robot_models::PointSphereBasicSE2Robot(initial_position,
                                                            position_distance_weight,
                                                            rotation_distance_weight,
                                                            link_name,
                                                            geometry),
              TnuvaRobot<simple_se2_robot_model::SimpleSE2Configuration, Generator>()
        {
            x_axis_controller_ = simple_pid_controller::SimplePIDController(robot_config.kp, robot_config.ki, robot_config.kd, robot_config.integral_clamp);
            y_axis_controller_ = simple_pid_controller::SimplePIDController(robot_config.kp, robot_config.ki, robot_config.kd, robot_config.integral_clamp);
            zr_axis_controller_ = simple_pid_controller::SimplePIDController(robot_config.r_kp, robot_config.r_ki, robot_config.r_kd, robot_config.r_integral_clamp);
            x_axis_sensor_ = simple_uncertainty_models::TruncatedNormalUncertainSensor(-robot_config.max_sensor_noise, robot_config.max_sensor_noise);
            y_axis_sensor_ = simple_uncertainty_models::TruncatedNormalUncertainSensor(-robot_config.max_sensor_noise, robot_config.max_sensor_noise);
            zr_axis_sensor_ = simple_uncertainty_models::TruncatedNormalUncertainSensor(-robot_config.r_max_sensor_noise, robot_config.r_max_sensor_noise);
            x_axis_actuator_ = simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator(robot_config.velocity_limit, robot_config.acceleration_limit, robot_config.max_actuator_proportional_noise, robot_config.max_actuator_minimum_noise, 0.5);
            y_axis_actuator_ = simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator(robot_config.velocity_limit, robot_config.acceleration_limit, robot_config.max_actuator_proportional_noise, robot_config.max_actuator_minimum_noise, 0.5);
            zr_axis_actuator_ = simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator(robot_config.r_velocity_limit, robot_config.r_acceleration_limit, robot_config.r_max_actuator_proportional_noise, robot_config.r_max_actuator_minimum_noise, 0.5);
            ResetControllers();
        }

        virtual simple_robot_model_interface::SimpleRobotModelInterface<simple_se2_robot_model::SimpleSE2Configuration, simple_se2_robot_model::SimpleSE2ConfigAlloc>* Clone() const
        {
            return new TnuvaSE2Robot(static_cast<const TnuvaSE2Robot>(*this));
        }

        virtual const simple_se2_robot_model::SimpleSE2Configuration& ResetPosition(const simple_se2_robot_model::SimpleSE2Configuration& position)
        {
            ResetControllers();
            return SetPosition(position);
        }

        virtual void ResetControllers()
        {
            x_axis_controller_.Zero();
            y_axis_controller_.Zero();
            zr_axis_controller_.Zero();
        }

        virtual void ApplyControlInput(const Eigen::VectorXd& input)
        {
            assert(input.size() == 3);
            Eigen::Matrix<double, 3, 1> real_input = Eigen::Matrix<double, 3, 1>::Zero();
            real_input(0) = x_axis_actuator_.GetControlValue(input(0));
            real_input(1) = y_axis_actuator_.GetControlValue(input(1));
            real_input(2) = zr_axis_actuator_.GetControlValue(input(2));
            // Compute new config
            const Eigen::Matrix<double, 3, 1> new_config = GetPosition() + real_input;
            // Update config
            SetPosition(new_config);
        }

        virtual void ApplyControlInput(const Eigen::VectorXd& input, Generator& rng)
        {
            assert(input.size() == 3);
            // Sense new noisy config
            Eigen::Matrix<double, 3, 1> noisy_input = Eigen::Matrix<double, 3, 1>::Zero();
            noisy_input(0) = x_axis_actuator_.GetControlValue(input(0), rng);
            noisy_input(1) = y_axis_actuator_.GetControlValue(input(1), rng);
            noisy_input(2) = zr_axis_actuator_.GetControlValue(input(2), rng);
            // Compute new config
            const Eigen::Matrix<double, 3, 1> new_config = GetPosition() + noisy_input;
            // Update config
            SetPosition(new_config);
        }

        virtual Eigen::VectorXd GenerateControlAction(const simple_se2_robot_model::SimpleSE2Configuration& target, const double controller_interval)
        {
            // Get the current position
            const Eigen::Matrix<double, 3, 1>& current = GetPosition();
            // Get the current error
            const Eigen::VectorXd current_error = ComputePerDimensionConfigurationSignedDistance(current, target);
            // Compute feedback terms
            const double x_term = x_axis_controller_.ComputeFeedbackTerm(current_error(0), controller_interval);
            const double y_term = y_axis_controller_.ComputeFeedbackTerm(current_error(1), controller_interval);
            const double zr_term = zr_axis_controller_.ComputeFeedbackTerm(current_error(2), controller_interval);
            // Make the control action
            const double x_axis_control = x_axis_actuator_.GetControlValue(x_term);
            const double y_axis_control = y_axis_actuator_.GetControlValue(y_term);
            const double zr_axis_control = zr_axis_actuator_.GetControlValue(zr_term);
            Eigen::VectorXd control_action(3);
            control_action(0) = x_axis_control;
            control_action(1) = y_axis_control;
            control_action(2) = zr_axis_control;
            return control_action;
        }
    };

    template<typename Generator>
    class TnuvaSE3Robot : public simple_robot_models::PointSphereBasicSE3Robot, TnuvaRobot<simple_se3_robot_model::SimpleSE3Configuration, Generator>
    {
    protected:

        simple_pid_controller::SimplePIDController x_axis_controller_;
        simple_pid_controller::SimplePIDController y_axis_controller_;
        simple_pid_controller::SimplePIDController z_axis_controller_;
        simple_pid_controller::SimplePIDController xr_axis_controller_;
        simple_pid_controller::SimplePIDController yr_axis_controller_;
        simple_pid_controller::SimplePIDController zr_axis_controller_;
        simple_uncertainty_models::TruncatedNormalUncertainSensor x_axis_sensor_;
        simple_uncertainty_models::TruncatedNormalUncertainSensor y_axis_sensor_;
        simple_uncertainty_models::TruncatedNormalUncertainSensor z_axis_sensor_;
        simple_uncertainty_models::TruncatedNormalUncertainSensor xr_axis_sensor_;
        simple_uncertainty_models::TruncatedNormalUncertainSensor yr_axis_sensor_;
        simple_uncertainty_models::TruncatedNormalUncertainSensor zr_axis_sensor_;
        simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator x_axis_actuator_;
        simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator y_axis_actuator_;
        simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator z_axis_actuator_;
        simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator xr_axis_actuator_;
        simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator yr_axis_actuator_;
        simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator zr_axis_actuator_;

    public:

        struct SE3_ROBOT_CONFIG
        {
            double kp;
            double ki;
            double kd;
            double integral_clamp;
            double velocity_limit;
            double acceleration_limit;
            double max_sensor_noise;
            double max_actuator_proportional_noise;
            double max_actuator_minimum_noise;
            double r_kp;
            double r_ki;
            double r_kd;
            double r_integral_clamp;
            double r_velocity_limit;
            double r_acceleration_limit;
            double r_max_sensor_noise;
            double r_max_actuator_proportional_noise;
            double r_max_actuator_minimum_noise;

            SE3_ROBOT_CONFIG()
            {
                kp = 0.0;
                ki = 0.0;
                kd = 0.0;
                integral_clamp = 0.0;
                velocity_limit = 0.0;
                acceleration_limit = 0.0;
                max_sensor_noise = 0.0;
                max_actuator_proportional_noise = 0.0;
                max_actuator_minimum_noise = 0.0;
                r_kp = 0.0;
                r_ki = 0.0;
                r_kd = 0.0;
                r_integral_clamp = 0.0;
                r_velocity_limit = 0.0;
                r_acceleration_limit = 0.0;
                r_max_sensor_noise = 0.0;
                r_max_actuator_proportional_noise = 0.0;
                r_max_actuator_minimum_noise = 0.0;
            }

            SE3_ROBOT_CONFIG(const double in_kp, const double in_ki, const double in_kd, const double in_integral_clamp, const double in_velocity_limit, const double in_acceleration_limit, const double in_max_sensor_noise, const double in_max_actuator_proportional_noise, const double in_max_actuator_minimum_noise, const double in_r_kp, const double in_r_ki, const double in_r_kd, const double in_r_integral_clamp, const double in_r_velocity_limit, const double in_r_acceleration_limit, const double in_r_max_sensor_noise, const double in_r_max_actuator_proportional_noise, const double in_r_max_actuator_minimum_noise)
            {
                kp = in_kp;
                ki = in_ki;
                kd = in_kd;
                integral_clamp = in_integral_clamp;
                velocity_limit = in_velocity_limit;
                acceleration_limit = in_acceleration_limit;
                max_sensor_noise = in_max_sensor_noise;
                max_actuator_proportional_noise = in_max_actuator_proportional_noise;
                max_actuator_minimum_noise = in_max_actuator_minimum_noise;
                r_kp = in_r_kp;
                r_ki = in_r_ki;
                r_kd = in_r_kd;
                r_integral_clamp = in_r_integral_clamp;
                r_velocity_limit = in_r_velocity_limit;
                r_acceleration_limit = in_r_acceleration_limit;
                r_max_sensor_noise = in_r_max_sensor_noise;
                r_max_actuator_proportional_noise = in_r_max_actuator_proportional_noise;
                r_max_actuator_minimum_noise = in_r_max_actuator_minimum_noise;
            }
        };

        TnuvaSE3Robot(const simple_se3_robot_model::SimpleSE3Configuration& initial_position,
                      const double position_distance_weight,
                      const double rotation_distance_weight,
                      const std::string& link_name,
                      const simple_robot_models::PointSphereGeometry& geometry,
                      const TnuvaSE3Robot::SE3_ROBOT_CONFIG& robot_config)
            : simple_robot_models::PointSphereBasicSE3Robot(initial_position,
                                                            position_distance_weight,
                                                            rotation_distance_weight,
                                                            link_name,
                                                            geometry),
              TnuvaRobot<simple_se3_robot_model::SimpleSE3Configuration, Generator>()
        {
            x_axis_controller_ = simple_pid_controller::SimplePIDController(robot_config.kp, robot_config.ki, robot_config.kd, robot_config.integral_clamp);
            y_axis_controller_ = simple_pid_controller::SimplePIDController(robot_config.kp, robot_config.ki, robot_config.kd, robot_config.integral_clamp);
            z_axis_controller_ = simple_pid_controller::SimplePIDController(robot_config.kp, robot_config.ki, robot_config.kd, robot_config.integral_clamp);
            xr_axis_controller_ = simple_pid_controller::SimplePIDController(robot_config.r_kp, robot_config.r_ki, robot_config.r_kd, robot_config.r_integral_clamp);
            yr_axis_controller_ = simple_pid_controller::SimplePIDController(robot_config.r_kp, robot_config.r_ki, robot_config.r_kd, robot_config.r_integral_clamp);
            zr_axis_controller_ = simple_pid_controller::SimplePIDController(robot_config.r_kp, robot_config.r_ki, robot_config.r_kd, robot_config.r_integral_clamp);
            x_axis_sensor_ = simple_uncertainty_models::TruncatedNormalUncertainSensor(-robot_config.max_sensor_noise, robot_config.max_sensor_noise);
            y_axis_sensor_ = simple_uncertainty_models::TruncatedNormalUncertainSensor(-robot_config.max_sensor_noise, robot_config.max_sensor_noise);
            z_axis_sensor_ = simple_uncertainty_models::TruncatedNormalUncertainSensor(-robot_config.max_sensor_noise, robot_config.max_sensor_noise);
            xr_axis_sensor_ = simple_uncertainty_models::TruncatedNormalUncertainSensor(-robot_config.r_max_sensor_noise, robot_config.r_max_sensor_noise);
            yr_axis_sensor_ = simple_uncertainty_models::TruncatedNormalUncertainSensor(-robot_config.r_max_sensor_noise, robot_config.r_max_sensor_noise);
            zr_axis_sensor_ = simple_uncertainty_models::TruncatedNormalUncertainSensor(-robot_config.r_max_sensor_noise, robot_config.r_max_sensor_noise);
            x_axis_actuator_ = simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator(robot_config.velocity_limit, robot_config.acceleration_limit, robot_config.max_actuator_proportional_noise, robot_config.max_actuator_minimum_noise, 0.5);
            y_axis_actuator_ = simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator(robot_config.velocity_limit, robot_config.acceleration_limit, robot_config.max_actuator_proportional_noise, robot_config.max_actuator_minimum_noise, 0.5);
            z_axis_actuator_ = simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator(robot_config.velocity_limit, robot_config.acceleration_limit, robot_config.max_actuator_proportional_noise, robot_config.max_actuator_minimum_noise, 0.5);
            xr_axis_actuator_ = simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator(robot_config.r_velocity_limit, robot_config.r_acceleration_limit, robot_config.r_max_actuator_proportional_noise, robot_config.r_max_actuator_minimum_noise, 0.5);
            yr_axis_actuator_ = simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator(robot_config.r_velocity_limit, robot_config.r_acceleration_limit, robot_config.r_max_actuator_proportional_noise, robot_config.r_max_actuator_minimum_noise, 0.5);
            zr_axis_actuator_ = simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator(robot_config.r_velocity_limit, robot_config.r_acceleration_limit, robot_config.r_max_actuator_proportional_noise, robot_config.r_max_actuator_minimum_noise, 0.5);
            ResetControllers();
        }

        virtual simple_robot_model_interface::SimpleRobotModelInterface<simple_se3_robot_model::SimpleSE3Configuration, simple_se3_robot_model::SimpleSE3ConfigAlloc>* Clone() const
        {
            return new TnuvaSE3Robot(static_cast<const TnuvaSE3Robot>(*this));
        }

        virtual const simple_se3_robot_model::SimpleSE3Configuration& ResetPosition(const simple_se3_robot_model::SimpleSE3Configuration& position)
        {
            ResetControllers();
            return SetPosition(position);
        }

        virtual void ResetControllers()
        {
            x_axis_controller_.Zero();
            y_axis_controller_.Zero();
            z_axis_controller_.Zero();
            xr_axis_controller_.Zero();
            yr_axis_controller_.Zero();
            zr_axis_controller_.Zero();
        }

        virtual void ApplyControlInput(const Eigen::VectorXd& input)
        {
            assert(input.size() == 6);
            // Sense new noisy config
            Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
            twist(0) = x_axis_actuator_.GetControlValue(input(0));
            twist(1) = y_axis_actuator_.GetControlValue(input(1));
            twist(2) = z_axis_actuator_.GetControlValue(input(2));
            twist(3) = xr_axis_actuator_.GetControlValue(input(3));
            twist(4) = yr_axis_actuator_.GetControlValue(input(4));
            twist(5) = zr_axis_actuator_.GetControlValue(input(5));
            // Compute the motion transform
            const Eigen::Isometry3d motion_transform = EigenHelpers::ExpTwist(twist, 1.0);
            const Eigen::Isometry3d new_config = GetPosition() * motion_transform;
            // Update config
            SetPosition(new_config);
        }

        virtual void ApplyControlInput(const Eigen::VectorXd& input, Generator& rng)
        {
            assert(input.size() == 6);
            // Sense new noisy config
            Eigen::Matrix<double, 6, 1> noisy_twist = Eigen::Matrix<double, 6, 1>::Zero();
            noisy_twist(0) = x_axis_actuator_.GetControlValue(input(0), rng);
            noisy_twist(1) = y_axis_actuator_.GetControlValue(input(1), rng);
            noisy_twist(2) = z_axis_actuator_.GetControlValue(input(2), rng);
            noisy_twist(3) = xr_axis_actuator_.GetControlValue(input(3), rng);
            noisy_twist(4) = yr_axis_actuator_.GetControlValue(input(4), rng);
            noisy_twist(5) = zr_axis_actuator_.GetControlValue(input(5), rng);
            // Compute the motion transform
            const Eigen::Isometry3d motion_transform = EigenHelpers::ExpTwist(noisy_twist, 1.0);
            const Eigen::Isometry3d new_config = GetPosition() * motion_transform;
            // Update config
            SetPosition(new_config);
        }

        virtual Eigen::VectorXd GenerateControlAction(const simple_se3_robot_model::SimpleSE3Configuration& target, const double controller_interval)
        {
            // Get the current position
            const Eigen::Isometry3d& current = GetPosition();
            // Get the twist from the current position to the target position
            const Eigen::Matrix<double, 6, 1> twist = EigenHelpers::TwistBetweenTransforms(current, target);
            // Compute feedback terms
            const double x_term = x_axis_controller_.ComputeFeedbackTerm(twist(0), controller_interval);
            const double y_term = y_axis_controller_.ComputeFeedbackTerm(twist(1), controller_interval);
            const double z_term = z_axis_controller_.ComputeFeedbackTerm(twist(2), controller_interval);
            const double xr_term = xr_axis_controller_.ComputeFeedbackTerm(twist(3), controller_interval);
            const double yr_term = yr_axis_controller_.ComputeFeedbackTerm(twist(4), controller_interval);
            const double zr_term = zr_axis_controller_.ComputeFeedbackTerm(twist(5), controller_interval);
            // Make the control action
            const double x_axis_control = x_axis_actuator_.GetControlValue(x_term);
            const double y_axis_control = y_axis_actuator_.GetControlValue(y_term);
            const double z_axis_control = z_axis_actuator_.GetControlValue(z_term);
            const double xr_axis_control = xr_axis_actuator_.GetControlValue(xr_term);
            const double yr_axis_control = yr_axis_actuator_.GetControlValue(yr_term);
            const double zr_axis_control = zr_axis_actuator_.GetControlValue(zr_term);
            Eigen::VectorXd control_action(6);
            control_action(0) = x_axis_control;
            control_action(1) = y_axis_control;
            control_action(2) = z_axis_control;
            control_action(3) = xr_axis_control;
            control_action(4) = yr_axis_control;
            control_action(5) = zr_axis_control;
            return control_action;
        }
    };

    template<typename Generator>
    class TnuvaLinkedRobot : public simple_robot_models::PointSphereBasicLinkedRobot, TnuvaRobot<simple_linked_robot_model::SimpleLinkedConfiguration, Generator>
    {
    public:

        struct LINKED_ROBOT_CONFIG
        {
            double kp;
            double ki;
            double kd;
            double integral_clamp;
            double velocity_limit;
            double acceleration_limit;
            double max_sensor_noise;
            double max_actuator_proportional_noise;
            double max_actuator_minimum_noise;

            LINKED_ROBOT_CONFIG()
            {
                kp = 0.0;
                ki = 0.0;
                kd = 0.0;
                integral_clamp = 0.0;
                velocity_limit = 0.0;
                acceleration_limit = 0.0;
                max_sensor_noise = 0.0;
                max_actuator_proportional_noise = 0.0;
                max_actuator_minimum_noise = 0.0;
            }

            LINKED_ROBOT_CONFIG(const double in_kp, const double in_ki, const double in_kd, const double in_integral_clamp, const double in_velocity_limit, const double in_acceleration_limit, const double in_max_sensor_noise, const double in_max_actuator_proportional_noise, const double in_max_actuator_minimum_noise)
            {
                kp = in_kp;
                ki = in_ki;
                kd = in_kd;
                integral_clamp = in_integral_clamp;
                velocity_limit = in_velocity_limit;
                acceleration_limit = in_acceleration_limit;
                max_sensor_noise = in_max_sensor_noise;
                max_actuator_proportional_noise = in_max_actuator_proportional_noise;
                max_actuator_minimum_noise = in_max_actuator_minimum_noise;
            }
        };

        struct JointControllerGroup
        {
            simple_pid_controller::SimplePIDController controller;
            simple_uncertainty_models::TruncatedNormalUncertainSensor sensor;
            simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator actuator;

            JointControllerGroup(const TnuvaLinkedRobot::LINKED_ROBOT_CONFIG& config)
            {
                controller = simple_pid_controller::SimplePIDController(config.kp, config.ki, config.kd, config.integral_clamp);
                sensor = simple_uncertainty_models::TruncatedNormalUncertainSensor(-config.max_sensor_noise, config.max_sensor_noise);
                actuator = simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator(config.velocity_limit, config.acceleration_limit, config.max_actuator_proportional_noise, config.max_actuator_minimum_noise, 0.5);
            }

            JointControllerGroup()
            {
                controller = simple_pid_controller::SimplePIDController(0.0, 0.0, 0.0, 0.0);
                sensor = simple_uncertainty_models::TruncatedNormalUncertainSensor(0.0, 0.0);
                actuator = simple_uncertainty_models::TruncatedNormalUncertainVelocityActuator(0.0, 0.0, 0.0, 0.0, 0.0);
            }
        };

    protected:

        std::vector<JointControllerGroup> joint_controller_groups_;

    public:

        TnuvaLinkedRobot(const Eigen::Isometry3d& base_transform,
                         const std::vector<simple_linked_robot_model::RobotLink>& links,
                         const std::vector<simple_linked_robot_model::RobotJoint>& joints,
                         const simple_linked_robot_model::SimpleLinkedConfiguration& initial_position,
                         const std::vector<double>& joint_distance_weights,
                         const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& link_geometries,
                         const std::vector<std::pair<size_t, size_t>>& allowed_self_collisions,
                         const std::vector<TnuvaLinkedRobot::LINKED_ROBOT_CONFIG>& joint_configs)
            : simple_robot_models::PointSphereBasicLinkedRobot(base_transform,
                                                               links,
                                                               joints,
                                                               initial_position,
                                                               joint_distance_weights,
                                                               link_geometries,
                                                               allowed_self_collisions),
              TnuvaRobot<simple_linked_robot_model::SimpleLinkedConfiguration, Generator>()
        {
            if (joint_configs.size() == this->num_active_joints_)
            {
                joint_controller_groups_.reserve(joint_configs.size());
                for (size_t idx = 0; idx < joint_configs.size(); idx++)
                {
                    joint_controller_groups_.push_back(TnuvaLinkedRobot::JointControllerGroup(joint_configs[idx]));
                }
                joint_controller_groups_.shrink_to_fit();
                ResetControllers();
            }
            else
            {
                throw std::invalid_argument("Number of joint configs must match number of active joints");
            }
        }

        virtual simple_robot_model_interface::SimpleRobotModelInterface<simple_linked_robot_model::SimpleLinkedConfiguration, simple_linked_robot_model::SimpleLinkedConfigAlloc>* Clone() const
        {
            return new TnuvaLinkedRobot(static_cast<const TnuvaLinkedRobot>(*this));
        }

        virtual const simple_linked_robot_model::SimpleLinkedConfiguration& ResetPosition(const simple_linked_robot_model::SimpleLinkedConfiguration& position)
        {
            ResetControllers();
            return SetPosition(position);
        }

        virtual void ResetControllers()
        {
            for (size_t idx = 0; idx < joint_controller_groups_.size(); idx++)
            {
                joint_controller_groups_[idx].controller.Zero();
            }
        }

        virtual void ApplyControlInput(const Eigen::VectorXd& input)
        {
            assert((size_t)input.size() == num_active_joints_);
            simple_linked_robot_model::SimpleLinkedConfiguration new_config;
            new_config.reserve(num_active_joints_);
            int64_t input_idx = 0u;
            for (size_t idx = 0; idx < joints_.size(); idx++)
            {
                const simple_linked_robot_model::RobotJoint& current_joint = joints_[idx];
                // Skip fixed joints
                if (current_joint.JointModel().IsFixed())
                {
                    continue;
                }
                else
                {
                    assert(input_idx < input.size());
                    const double input_val = input(input_idx);
                    const double real_input_val = joint_controller_groups_[input_idx].actuator.GetControlValue(input_val);
                    const double current_val = current_joint.JointModel().GetValue();
                    const double raw_new_val = current_val + real_input_val;
                    new_config.push_back(current_joint.JointModel().CopyWithNewValue(raw_new_val));
                    input_idx++;
                }
            }
            new_config.shrink_to_fit();
            // Update config
            SetPosition(new_config);
        }

        virtual void ApplyControlInput(const Eigen::VectorXd& input, Generator& rng)
        {
            assert((size_t)input.size() == num_active_joints_);
            simple_linked_robot_model::SimpleLinkedConfiguration new_config;
            new_config.reserve(num_active_joints_);
            int64_t input_idx = 0u;
            for (size_t idx = 0; idx < joints_.size(); idx++)
            {
                const simple_linked_robot_model::RobotJoint& current_joint = joints_[idx];
                // Skip fixed joints
                if (current_joint.JointModel().IsFixed())
                {
                    continue;
                }
                else
                {
                    assert(input_idx < input.size());
                    const double input_val = input(input_idx);
                    const double noisy_input_val = joint_controller_groups_[input_idx].actuator.GetControlValue(input_val, rng);
                    const double current_val = current_joint.JointModel().GetValue();
                    const double noisy_new_val = current_val + noisy_input_val;
                    new_config.push_back(current_joint.JointModel().CopyWithNewValue(noisy_new_val));
                    input_idx++;
                }
            }
            new_config.shrink_to_fit();
            // Update config
            SetPosition(new_config);
        }

        virtual Eigen::VectorXd GenerateControlAction(const simple_linked_robot_model::SimpleLinkedConfiguration& target, const double controller_interval)
        {
            // Get the current position
            const simple_linked_robot_model::SimpleLinkedConfiguration& current = GetPosition();
            // Get the current error
            const Eigen::VectorXd current_error = this->ComputeUnweightedPerDimensionConfigurationRawDistance(current, target);
            // Make the control action
            Eigen::VectorXd control_action = Eigen::VectorXd::Zero(current_error.size());
            for (size_t idx = 0; idx < joint_controller_groups_.size(); idx++)
            {
                const double joint_error = current_error((ssize_t)idx);
                const double joint_term = joint_controller_groups_[idx].controller.ComputeFeedbackTerm(joint_error, controller_interval);
                const double joint_control = joint_controller_groups_[idx].actuator.GetControlValue(joint_term);
                control_action((ssize_t)idx) = joint_control;
            }
            return control_action;
        }
    };
}

#endif // TNUVA_ROBOT_MODELS_HPP
