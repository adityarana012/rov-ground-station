#pragma once

#include <algorithm>
#include <cmath>

namespace rov {

/// A small PID controller for the hold modes.
///
/// Three choices worth defending at interview:
///
/// 1. Derivative on measurement, not error. The D term is driven by the
///    vehicle's own measured rate (depth rate, yaw rate) rather than by the
///    error derivative. A step change in the setpoint therefore produces a
///    smooth proportional response instead of a derivative spike that would
///    briefly saturate the thrusters. The caller passes the measurement rate
///    in explicitly, which keeps the controller unaware of time derivatives.
///
/// 2. Clamping anti-windup. The integrator only accumulates when the output is
///    not already saturated. Without this, a long approach to the setpoint
///    (say, a dive from the surface to 10 m with heave thrust saturated the
///    whole way) builds up an integrator value that then has to be discharged
///    by overshooting the other side of the setpoint before the controller
///    will reduce thrust - the classic "integral windup" overshoot.
///
/// 3. The controller is reset on mode switch, not carried across. See
///    Simulator::setMode().
class Pid {
public:
    Pid() = default;

    void setGains(double kp, double ki, double kd)
    {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
    }

    void setOutputLimits(double minOut, double maxOut)
    {
        m_minOut = minOut;
        m_maxOut = maxOut;
    }

    void reset()
    {
        m_integral = 0.0;
    }

    /// Compute the controller output.
    /// \param error        setpoint minus measurement
    /// \param measureRate  the time derivative of the measurement (not of error)
    /// \param dt           timestep in seconds
    double update(double error, double measureRate, double dt)
    {
        // Provisional integrator, before we know whether the output will saturate.
        const double trialI = m_integral + error * dt;

        // D on measurement: when the vehicle is moving in the +measurement
        // direction, that counts as negative error rate, hence the sign.
        const double unclamped = m_kp * error
                               + m_ki * trialI
                               - m_kd * measureRate;

        const double output = std::clamp(unclamped, m_minOut, m_maxOut);

        // Anti-windup: only commit the integrator step if it did not push the
        // output past its limits, or if it is working to bring the output back
        // inside them. This is the conditional-integration rule - simple,
        // robust, and the standard answer to "how did you handle windup?".
        const bool saturatedHigh = (unclamped > m_maxOut) && (error > 0.0);
        const bool saturatedLow  = (unclamped < m_minOut) && (error < 0.0);
        if (!saturatedHigh && !saturatedLow)
            m_integral = trialI;

        return output;
    }

    double integral() const { return m_integral; }

private:
    double m_kp = 0.0;
    double m_ki = 0.0;
    double m_kd = 0.0;

    double m_minOut = -1.0;
    double m_maxOut =  1.0;

    double m_integral = 0.0;
};

/// Return \p angle normalised into the half-open interval [-180, 180).
/// Used by the heading controller so that a setpoint of 10 and a measurement
/// of 350 produce an error of +20, not -340, and the vehicle turns the short
/// way around the compass.
inline double wrapError180(double degrees)
{
    degrees = std::fmod(degrees + 180.0, 360.0);
    if (degrees < 0.0)
        degrees += 360.0;
    return degrees - 180.0;
}

} // namespace rov
