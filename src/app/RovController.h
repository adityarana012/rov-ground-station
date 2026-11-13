#pragma once

#include <QElapsedTimer>
#include <QObject>
#include <QString>
#include <QTimer>
#include <QtQml/qqmlregistration.h>

#include "CsvLogger.h"
#include "Simulator.h"

namespace rov {

/// QML bridge over the headless Simulator.
///
/// This is deliberately the ONLY class that both knows about Qt and knows about
/// the simulator. Nothing in the simulator layer includes a Qt header and
/// nothing in QML reaches past this object. A future MAVLink replacement would
/// re-implement this class without touching either side.
///
/// Threading: the simulator is ticked here, on the GUI thread, from a QTimer.
/// The physics is cheap - a 50 Hz tick is on the order of microseconds - so a
/// background thread buys complexity without buying responsiveness. Should the
/// physics ever get heavy, the clean seam is to move the Simulator onto a
/// QThread and queue-connect the demand setters; no QML would change.
class RovController : public QObject {
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

    // --- Status ------------------------------------------------------------
    Q_PROPERTY(bool    armed       READ armed       NOTIFY telemetryChanged)
    Q_PROPERTY(int     mode        READ mode        NOTIFY telemetryChanged)
    Q_PROPERTY(QString modeName    READ modeName    NOTIFY telemetryChanged)

    // --- Attitude ----------------------------------------------------------
    Q_PROPERTY(double  roll        READ roll        NOTIFY telemetryChanged)
    Q_PROPERTY(double  pitch       READ pitch       NOTIFY telemetryChanged)
    Q_PROPERTY(double  yaw         READ yaw         NOTIFY telemetryChanged)

    // --- Position ----------------------------------------------------------
    Q_PROPERTY(double  depth       READ depth       NOTIFY telemetryChanged)
    Q_PROPERTY(double  north       READ north       NOTIFY telemetryChanged)
    Q_PROPERTY(double  east        READ east        NOTIFY telemetryChanged)

    // --- Power -------------------------------------------------------------
    Q_PROPERTY(double  batteryVolts    READ batteryVolts    NOTIFY telemetryChanged)
    Q_PROPERTY(double  batteryFraction READ batteryFraction NOTIFY telemetryChanged)
    Q_PROPERTY(double  batteryCurrent  READ batteryCurrent  NOTIFY telemetryChanged)

    // --- Thrust (used by the dashboard visual indicators) -----------------
    Q_PROPERTY(double  surgeThrust READ surgeThrust NOTIFY telemetryChanged)
    Q_PROPERTY(double  swayThrust  READ swayThrust  NOTIFY telemetryChanged)
    Q_PROPERTY(double  heaveThrust READ heaveThrust NOTIFY telemetryChanged)
    Q_PROPERTY(double  yawThrust   READ yawThrust   NOTIFY telemetryChanged)

    // --- Setpoints ----------------------------------------------------------
    Q_PROPERTY(double  targetDepth   READ targetDepth   NOTIFY telemetryChanged)
    Q_PROPERTY(double  targetHeading READ targetHeading NOTIFY telemetryChanged)

    // --- Ocean current (world-frame m/s) -----------------------------------
    Q_PROPERTY(double  currentSpeed   READ currentSpeed   NOTIFY telemetryChanged)
    Q_PROPERTY(double  currentHeading READ currentHeading NOTIFY telemetryChanged)
    Q_PROPERTY(double  currentDown    READ currentDown    NOTIFY telemetryChanged)

    // --- Logging ----------------------------------------------------------
    Q_PROPERTY(bool    logging    READ logging    NOTIFY telemetryChanged)
    Q_PROPERTY(QString logPath    READ logPath    NOTIFY telemetryChanged)
    Q_PROPERTY(int     logRows    READ logRows    NOTIFY telemetryChanged)

    // --- Housekeeping ------------------------------------------------------
    Q_PROPERTY(double  missionTime      READ missionTime      NOTIFY telemetryChanged)
    Q_PROPERTY(double  timeSinceInput   READ timeSinceInput   NOTIFY telemetryChanged)

public:
    explicit RovController(QObject* parent = nullptr);

    // --- Property getters --------------------------------------------------
    bool    armed() const           { return m_sim.state().armed; }
    int     mode() const            { return int(m_sim.state().mode); }
    QString modeName() const;

    double  roll() const            { return m_sim.state().roll; }
    double  pitch() const           { return m_sim.state().pitch; }
    double  yaw() const             { return m_sim.state().yaw; }

    double  depth() const           { return m_sim.state().depth; }
    double  north() const           { return m_sim.state().north; }
    double  east() const            { return m_sim.state().east; }

    double  batteryVolts() const    { return m_sim.state().batteryVolts; }
    double  batteryFraction() const { return m_sim.batteryFraction(); }
    double  batteryCurrent() const  { return m_sim.batteryCurrent(); }

    double  surgeThrust() const     { return m_sim.state().surgeThrust; }
    double  swayThrust() const      { return m_sim.state().swayThrust; }
    double  heaveThrust() const     { return m_sim.state().heaveThrust; }
    double  yawThrust() const       { return m_sim.state().yawThrust; }

    double  targetDepth() const     { return m_sim.state().targetDepth; }
    double  targetHeading() const   { return m_sim.state().targetHeading; }

    double  missionTime() const     { return m_sim.state().missionTime; }
    double  timeSinceInput() const  { return m_sim.timeSinceInput(); }

    double  currentSpeed() const;
    double  currentHeading() const;
    double  currentDown() const     { return m_sim.currentDown(); }

    bool    logging() const         { return m_logger.isOpen(); }
    QString logPath() const
        { return QString::fromStdString(m_logger.path()); }
    int     logRows() const
        { return int(m_logger.rowCount()); }

    // --- QML-callable commands --------------------------------------------
    Q_INVOKABLE void toggleArm();
    Q_INVOKABLE void setArmed(bool armed);

    /// \p mode matches the rov::Mode enum: 0 = Manual, 1 = DepthHold,
    ///                                     2 = HeadingHold.
    Q_INVOKABLE void selectMode(int mode);

    Q_INVOKABLE void setTargetDepth(double metres);
    Q_INVOKABLE void setTargetHeading(double degrees);

    /// Supply a new Manual-mode demand. Resets the dead-man timer.
    /// Only takes effect while the vehicle is in Manual mode - the hold modes
    /// override their respective axes in the simulator.
    Q_INVOKABLE void setManualDemand(double surge, double sway,
                                     double heave, double yaw);

    /// Called by the keyboard driver on each key change in Main.qml. The
    /// controller translates WASD/QE/RF into a demand vector itself, so the
    /// QML side only has to report which keys are currently down.
    Q_INVOKABLE void setKeyState(bool wDown, bool sDown,
                                 bool aDown, bool dDown,
                                 bool qDown, bool eDown,
                                 bool rDown, bool fDown);

    /// Set the ocean current as speed (m/s) + direction (deg, the direction
    /// the current flows TOWARD, measured clockwise from north). Also sets
    /// the vertical component (positive = downward).
    Q_INVOKABLE void setCurrent(double speed, double headingDeg, double down);

    /// Returns the path the log was actually opened at, or an empty string on
    /// failure. If \p path is empty, writes to ~/rov_YYYYMMDD_HHMMSS.csv.
    Q_INVOKABLE QString startLogging(const QString& path = QString());
    Q_INVOKABLE void    stopLogging();

    Q_INVOKABLE void resetSim();

    // Only needed by the depth-plot scaffold wired up in Phase 4.
    Q_INVOKABLE double fixedStepSeconds() const
        { return Simulator::kFixedStep; }

signals:
    /// Emitted once per simulator tick (50 Hz). Every Q_PROPERTY on this class
    /// uses it as its NOTIFY signal: QML only has to listen for one thing.
    void telemetryChanged();

protected:
    /// Application-wide key event filter. Keyboard focus in QML is fragile:
    /// clicking a SpinBox moves focus to its TextInput, which then eats every
    /// subsequent WASD/RF press. Filtering at the application level sidesteps
    /// the whole focus dance - the only check needed is "is a text widget
    /// being typed into right now?", in which case we let the key through.
    bool eventFilter(QObject* watched, QEvent* event) override;

private slots:
    void tick();

private:
    void updateKeyDemand();

    Simulator m_sim;

    struct KeyState {
        bool w = false, s = false, a = false, d = false;
        bool q = false, e = false, r = false, f = false;
    } m_keys;

    ControlInput m_keyDemand;
    bool m_anyKeyDown = false;

    QTimer m_timer;
    QElapsedTimer m_elapsed;
    qint64 m_lastTickNs = 0;

    CsvLogger m_logger;
    double m_nextLogAt = 0.0;   ///< next mission-time to emit a log row
    static constexpr double kLogIntervalSeconds = 0.1;   // 10 Hz
};

} // namespace rov
