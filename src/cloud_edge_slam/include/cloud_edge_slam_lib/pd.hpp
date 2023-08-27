/// Standard PD (proportional, derivative) controller. Derivative
/// component is filtered using an exponential moving average filter.
class PD {
  public:
    PD() = default;
    /// @param  kp
    ///         Proportional gain
    /// @param  kd
    ///         Derivative gain
    /// @param  Ts
    ///         Sampling time (seconds)

    PD(float kp, float kd, float Alpha = 1, float Ts = 1, float maxOutput = 255)
        : Ts(Ts), maxOutput(maxOutput), Alpha(Alpha) {
        setKp(kp);
        setKd(kd);
    }

    /// Update the controller: given the current position, compute the control
    /// action.
    float update(float input, double Ts) {
        // The error is the difference between the reference (setpoint) and the
        // actual position (input)
        float error = setpoint - input;
        // Compute the difference between the current and the previous input,
        // but compute a weighted average using a factor α ∊ (0,1]
        float diff = Alpha * (prevInput - input);
        // Update the average
        prevInput -= diff;

        // Standard PID rule
        float output = kp * error + kd /Ts * diff;

        // Clamp and anti-windup
        if (output > maxOutput)
            output = maxOutput;

        return output;
    }

    void setKp(float kp) { this->kp = kp; }               ///< Proportional gain
    void setKd(float kd) { this->kd = kd; } ///< Derivative gain

    float getKp() const { return kp; }         ///< Proportional gains
    float getKd() const { return kd; } ///< Derivative gain

    /// Set the cutoff frequency (-3 dB point) of the exponential moving average
    /// filter that is applied to the input before taking the difference for
    /// computing the derivative term.

    /// Set the reference/target/setpoint of the controller.
    void setSetpoint(float setpoint) {
        this->setpoint = setpoint;
    }
    
    float getSetpoint() const { return setpoint; }

    /// Set the maximum control output magnitude. Default is 255, which clamps
    /// the control output in [-255, +255].
    void setMaxOutput(float maxOutput) { this->maxOutput = maxOutput; }
    /// @see @ref setMaxOutput(float)
    float getMaxOutput() const { return maxOutput; }

  private:
    float Ts = 1;               ///< Sampling time (seconds)
    float maxOutput = 15;      ///< Maximum control output magnitude
    float kp = 1;               ///< Proportional gain
    float kd = 0;               ///< Derivative gain divided by Ts
    float prevInput = 0;        ///< (Filtered) previous input for derivative.
    float setpoint = 0;      ///< Position reference.
    float Alpha = 1;
};