#pragma once

#include "VehicleState.h"

#include <fstream>
#include <string>

namespace rov {

/// Mission telemetry logger.
///
/// Writes one CSV row per call to write(), in a schema designed to be loaded
/// directly into a spreadsheet or into pandas without any post-processing.
/// The columns match the fields a post-mission analyst would want: the raw
/// state, the actuator outputs, the controller setpoints, and the battery.
///
/// Deliberately uses only standard C++ I/O. The simulator layer is Qt-free and
/// stays that way; the GUI simply tells the logger when to start and stop.
class CsvLogger {
public:
    ~CsvLogger() { close(); }

    /// Open \p path for writing. If a file is already open it is closed first.
    /// On success the header row is written and the logger is ready for rows.
    bool open(const std::string& path);
    void close();

    bool isOpen() const { return m_stream.is_open(); }
    const std::string& path() const { return m_path; }
    std::size_t rowCount() const { return m_rows; }

    /// Append one row.
    void write(double missionTime,
               const VehicleState& state,
               double batteryCurrent,
               double batteryFraction);

private:
    std::ofstream m_stream;
    std::string m_path;
    std::size_t m_rows = 0;
};

} // namespace rov
