#include "CsvLogger.h"

#include <iomanip>

namespace rov {
namespace {

const char* modeName(Mode m)
{
    switch (m) {
        case Mode::Manual:      return "MANUAL";
        case Mode::DepthHold:   return "DEPTH_HOLD";
        case Mode::HeadingHold: return "HEADING_HOLD";
    }
    return "UNKNOWN";
}

} // namespace

bool CsvLogger::open(const std::string& path)
{
    close();

    m_stream.open(path);
    if (!m_stream.is_open())
        return false;

    m_path = path;
    m_rows = 0;

    // Fixed-point output keeps the columns the same width per row, which
    // matters when the file is tailed live during a mission.
    m_stream << std::fixed << std::setprecision(3);

    m_stream <<
        "time_s,mode,armed,"
        "roll_deg,pitch_deg,yaw_deg,"
        "depth_m,north_m,east_m,"
        "surge_vel_mps,sway_vel_mps,heave_vel_mps,yaw_rate_dps,"
        "surge_thrust,sway_thrust,heave_thrust,yaw_thrust,"
        "battery_v,battery_a,battery_fraction,"
        "target_depth_m,target_heading_deg\n";

    return m_stream.good();
}

void CsvLogger::close()
{
    if (m_stream.is_open()) {
        m_stream.flush();
        m_stream.close();
    }
    m_path.clear();
}

void CsvLogger::write(double t, const VehicleState& s,
                      double batteryCurrent, double batteryFraction)
{
    if (!m_stream.is_open())
        return;

    m_stream
        << t                   << ','
        << modeName(s.mode)    << ','
        << (s.armed ? 1 : 0)   << ','
        << s.roll              << ','
        << s.pitch             << ','
        << s.yaw               << ','
        << s.depth             << ','
        << s.north             << ','
        << s.east              << ','
        << s.surgeVel          << ','
        << s.swayVel           << ','
        << s.heaveVel          << ','
        << s.yawRate           << ','
        << s.surgeThrust       << ','
        << s.swayThrust        << ','
        << s.heaveThrust       << ','
        << s.yawThrust         << ','
        << s.batteryVolts      << ','
        << batteryCurrent      << ','
        << batteryFraction     << ','
        << s.targetDepth       << ','
        << s.targetHeading     << '\n';

    ++m_rows;
}

} // namespace rov
