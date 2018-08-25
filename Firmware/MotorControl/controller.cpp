
#include "odrive_main.h"


Controller::Controller(ControllerConfig_t& config) :
    config_(config)
{}

void Controller::reset() {
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    vel_integrator_current_ = 0.0f;
    current_setpoint_ = 0.0f;
}

//--------------------------------
// Command Handling
//--------------------------------

void Controller::set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward) {
    pos_setpoint_ = pos_setpoint;
    vel_setpoint_ = vel_feed_forward;
    current_setpoint_ = current_feed_forward;
    config_.control_mode = CTRL_MODE_POSITION_CONTROL;
#ifdef DEBUG_PRINT
    printf("POSITION_CONTROL %6.0f %3.3f %3.3f\n", pos_setpoint, vel_setpoint_, current_setpoint_);
#endif
}

void Controller::set_vel_setpoint(float vel_setpoint, float current_feed_forward) {
    vel_setpoint_ = vel_setpoint;
    current_setpoint_ = current_feed_forward;
    config_.control_mode = CTRL_MODE_VELOCITY_CONTROL;
#ifdef DEBUG_PRINT
    printf("VELOCITY_CONTROL %3.3f %3.3f\n", vel_setpoint_, motor->current_setpoint_);
#endif
}

void Controller::set_current_setpoint(float current_setpoint) {
    current_setpoint_ = current_setpoint;
    config_.control_mode = CTRL_MODE_CURRENT_CONTROL;
#ifdef DEBUG_PRINT
    printf("CURRENT_CONTROL %3.3f\n", current_setpoint_);
#endif
}

void Controller::set_coupled_setpoints(float theta_setpoint, float gamma_setpoint) {
    theta_setpoint_ = theta_setpoint;
    gamma_setpoint_ = gamma_setpoint;
    config_.control_mode = CTRL_MODE_COUPLED_CONTROL;
#ifdef DEBUG_PRINT
    printf("COUPLED_CONTROL %3.3f %3.3f\n", theta_setpoint_, gamma_setpoint_);
#endif
}

float Controller::encoder_to_rad(float x) {
    return x / (axis_->encoder_.config_.cpr * config_.gear_ratio) * 2 * M_PI;
}

void Controller::start_anticogging_calibration() {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (anticogging_.cogging_map != NULL && axis_->error_ == Axis::ERROR_NONE) {
        anticogging_.calib_anticogging = true;
    }
}

/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 *
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    if (anticogging_.calib_anticogging && anticogging_.cogging_map != NULL) {
        float pos_err = anticogging_.index - pos_estimate;
        if (fabsf(pos_err) <= anticogging_.calib_pos_threshold &&
            fabsf(vel_estimate) < anticogging_.calib_vel_threshold) {
            anticogging_.cogging_map[anticogging_.index++] = vel_integrator_current_;
        }
        if (anticogging_.index < axis_->encoder_.config_.cpr) { // TODO: remove the dependency on encoder CPR
            set_pos_setpoint(anticogging_.index, 0.0f, 0.0f);
            return false;
        } else {
            anticogging_.index = 0;
            set_pos_setpoint(0.0f, 0.0f, 0.0f);  // Send the motor home
            anticogging_.use_anticogging = true;  // We're good to go, enable anti-cogging
            anticogging_.calib_anticogging = false;
            return true;
        }
    }
    return false;
}

bool Controller::update(float pos_estimate, float vel_estimate, float* current_setpoint_output) {
    // Only runs if anticogging_.calib_anticogging is true; non-blocking
    anticogging_calibration(pos_estimate, vel_estimate);

    // PD control
    float Iq = 0.0f;

    if (config_.control_mode == CTRL_MODE_POSITION_CONTROL) {
        float pos_err = pos_setpoint_ - pos_estimate;
        Iq += config_.pos_gain * pos_err;

        float vel_err = 0 - vel_estimate;
        Iq += config_.vel_gain * vel_err;
    }

    // Coupled PD control
    if (config_.control_mode == CTRL_MODE_COUPLED_CONTROL) {
      float alpha = encoder_to_rad(axes[0]->encoder_.pos_estimate_);
      float beta = encoder_to_rad(axes[1]->encoder_.pos_estimate_) + M_PI; // Assumes legs started 180 apart
      float d_alpha = encoder_to_rad(axes[0]->encoder_.pll_vel_);
      float d_beta = encoder_to_rad(axes[1]->encoder_.pll_vel_);

      float theta = alpha/2.0 + beta/2.0;
      float gamma = beta/2.0 - alpha/2.0;
      float d_theta = d_alpha/2.0 + d_beta/2.0;
      float d_gamma = d_beta/2.0 - d_alpha/2.0;

      float p_term_theta = config_.kp_theta * (theta_setpoint_ - theta);
      float d_term_theta = config_.kd_theta * (0 - d_theta);

      float p_term_gamma = config_.kp_gamma * (gamma_setpoint_ - gamma);
      float d_term_gamma = config_.kd_gamma * (0 - d_gamma);

      float tau_theta = p_term_theta + d_term_theta;
      float tau_gamma = p_term_gamma + d_term_gamma;

      axes[0]->controller_.current_setpoint_ = tau_theta*0.5 - tau_gamma*0.5;
      axes[1]->controller_.current_setpoint_ = tau_theta*0.5 + tau_gamma*0.5;

      Iq = current_setpoint_;
    }

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_.use_anticogging) {
        Iq += anticogging_.cogging_map[mod(static_cast<int>(pos_estimate), axis_->encoder_.config_.cpr)];
    }

    // Current limiting
    float Ilim = std::min(axis_->motor_.config_.current_lim, axis_->motor_.current_control_.max_allowed_current);
    bool limited = false;
    if (Iq > Ilim) {
        limited = true;
        Iq = Ilim;
    }
    if (Iq < -Ilim) {
        limited = true;
        Iq = -Ilim;
    }

    if (current_setpoint_output) *current_setpoint_output = Iq;
    return true;
}
