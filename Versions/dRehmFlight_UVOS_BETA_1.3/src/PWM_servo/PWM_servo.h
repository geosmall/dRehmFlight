#pragma once

//This file provides a PWMservo emulation interface to UVOS Timers for PWM out

namespace uvos {

class PWM_servo {
public:
	PWM_servo(PWMOutput& pwm_output, size_t output_index)
		: pwm_output_(pwm_output),
		  output_index_(output_index),
		  min_pulse_width_(900),
		  max_pulse_width_(2100),
		  min_angle_(40),
		  max_angle_(140) {}

	/** @brief Sets the minimum and maximum pulse widths for the servo.
	 *  @param min_pulse_width Minimum pulse width in microseconds (0 degrees).
	 *  @param max_pulse_width Maximum pulse width in microseconds (180 degrees).
	 */
	void limit(uint32_t min_pulse_width, uint32_t max_pulse_width) {
		min_pulse_width_ = min_pulse_width;
		max_pulse_width_ = max_pulse_width;
	}

	/** @brief Sets the minimum and maximum angles for the servo.
	 *  @param min_angle Minimum angle in degrees.
	 *  @param max_angle Maximum angle in degrees.
	 */
	void setAngleRange(int min_angle, int max_angle) {
		min_angle_ = min_angle;
		max_angle_ = max_angle;
	}

	/** @brief Writes an angle to the servo, moving it to the specified position.
	 *  @param angle Angle in degrees.
	 */
	void write(int angle) {
		angle = std::clamp(angle, min_angle_, max_angle_);
		uint32_t pulse_width = mapAngleToPulseWidth(angle);
		pwm_output_.SetPulseWidth(output_index_, pulse_width);
	}

	/** @brief Writes a pulse width directly to the servo.
	 *  @param pulse_width Pulse width in microseconds.
	 */
	void writeMicroseconds(uint32_t pulse_width) {
		pulse_width = std::clamp(pulse_width, min_pulse_width_, max_pulse_width_);
		pwm_output_.SetPulseWidth(output_index_, pulse_width);
	}

private:
	PWMOutput& pwm_output_;
	size_t output_index_;
	uint32_t min_pulse_width_; // Pulse width corresponding to min_angle_
	uint32_t max_pulse_width_; // Pulse width corresponding to max_angle_
	int min_angle_;
	int max_angle_;

	/** @brief Maps an angle to a pulse width based on the min and max pulse widths and angles.
	 *  @param angle Angle in degrees.
	 *  @return Pulse width in microseconds.
	 */
	uint32_t mapAngleToPulseWidth(int angle) {
		// Linear mapping from angle to pulse width
		return min_pulse_width_ + ((max_pulse_width_ - min_pulse_width_) * (angle - min_angle_)) / (max_angle_ - min_angle_);
	}
};

class PWM_esc {
public:
	PWM_esc(PWMOutput& pwm_output, size_t output_index)
		: pwm_output_(pwm_output), output_index_(output_index) {}

	/** @brief Writes a pulse width value to the ESC.
	 *  @param pulse_width Pulse width in microseconds.
	 */
	void write(uint32_t pulse_width) {
		pwm_output_.SetPulseWidth(output_index_, pulse_width);
	}

private:
	PWMOutput& pwm_output_;
	size_t output_index_;
};

} // namespace uvos
