#include "RovController.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QEvent>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QMetaObject>
#include <QStandardPaths>

#include <cmath>
#include <cstring>

namespace rov {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadPerDeg = kPi / 180.0;
constexpr double kDegPerRad = 180.0 / kPi;
} // namespace

RovController::RovController(QObject* parent)
    : QObject(parent)
{
    // Tick as close to the physics rate as the Qt timer resolution allows. The
    // tick() slot measures the real elapsed time and hands that to step(), so
    // any jitter or late frame is compensated for inside the simulator's
    // fixed-step accumulator rather than silently lost.
    m_timer.setTimerType(Qt::PreciseTimer);
    m_timer.setInterval(20);   // 50 Hz
    QObject::connect(&m_timer, &QTimer::timeout, this, &RovController::tick);

    m_elapsed.start();
    m_lastTickNs = m_elapsed.nsecsElapsed();
    m_timer.start();

    // Pick up keyboard input regardless of which QML widget currently has
    // focus. See the header comment on eventFilter() for why this beats a
    // focus-based approach.
    if (QCoreApplication::instance())
        QCoreApplication::instance()->installEventFilter(this);
}

QString RovController::modeName() const
{
    switch (m_sim.state().mode) {
        case Mode::Manual:      return QStringLiteral("MANUAL");
        case Mode::DepthHold:   return QStringLiteral("DEPTH HOLD");
        case Mode::HeadingHold: return QStringLiteral("HEADING HOLD");
    }
    return QStringLiteral("?");
}

void RovController::toggleArm()
{
    m_sim.setArmed(!m_sim.state().armed);
    emit telemetryChanged();
}

void RovController::setArmed(bool armed)
{
    m_sim.setArmed(armed);
    emit telemetryChanged();
}

void RovController::selectMode(int mode)
{
    if (mode < 0 || mode > int(Mode::HeadingHold))
        return;
    m_sim.setMode(static_cast<Mode>(mode));
    emit telemetryChanged();
}

void RovController::setTargetDepth(double metres)
{
    m_sim.setTargetDepth(metres);
    emit telemetryChanged();
}

void RovController::setTargetHeading(double degrees)
{
    m_sim.setTargetHeading(degrees);
    emit telemetryChanged();
}

void RovController::setManualDemand(double surge, double sway,
                                    double heave, double yaw)
{
    m_sim.setControlInput(ControlInput{surge, sway, heave, yaw});
}

void RovController::setKeyState(bool wDown, bool sDown,
                                bool aDown, bool dDown,
                                bool qDown, bool eDown,
                                bool rDown, bool fDown)
{
    // Legacy QML entry point. The app-level event filter is the real path now;
    // this slot is kept so a QML harness or test can still drive the vehicle.
    m_keys = KeyState{wDown, sDown, aDown, dDown, qDown, eDown, rDown, fDown};
    updateKeyDemand();
}

void RovController::updateKeyDemand()
{
    m_keyDemand.surge = (m_keys.w ? 1.0 : 0.0) + (m_keys.s ? -1.0 : 0.0);
    m_keyDemand.sway  = (m_keys.d ? 1.0 : 0.0) + (m_keys.a ? -1.0 : 0.0);
    m_keyDemand.yaw   = (m_keys.e ? 1.0 : 0.0) + (m_keys.q ? -1.0 : 0.0);
    m_keyDemand.heave = (m_keys.f ? 1.0 : 0.0) + (m_keys.r ? -1.0 : 0.0);

    m_anyKeyDown = m_keys.w || m_keys.s || m_keys.a || m_keys.d
                || m_keys.q || m_keys.e || m_keys.r || m_keys.f;
}

bool RovController::eventFilter(QObject* watched, QEvent* event)
{
    const QEvent::Type t = event->type();
    if (t != QEvent::KeyPress && t != QEvent::KeyRelease)
        return QObject::eventFilter(watched, event);

    auto* k = static_cast<QKeyEvent*>(event);
    if (k->isAutoRepeat())
        return false;   // let the OS auto-repeat through without changing state

    // Don't steal keystrokes from a text field a human is typing into.
    // SpinBox's editable child is a QQuickTextInput; the inline editor on an
    // editable ComboBox is a QQuickTextField. We match by class name so we
    // don't have to link QtQuick's private headers.
    if (QObject* focused = QCoreApplication::instance()
                               ? QGuiApplication::focusObject() : nullptr) {
        const char* cls = focused->metaObject()->className();
        if (cls && (std::strstr(cls, "TextInput") || std::strstr(cls, "TextEdit")
                    || std::strstr(cls, "TextField") || std::strstr(cls, "TextArea")))
            return false;
    }

    const bool down = (t == QEvent::KeyPress);
    bool handled = true;

    switch (k->key()) {
        case Qt::Key_W: m_keys.w = down; updateKeyDemand(); break;
        case Qt::Key_S: m_keys.s = down; updateKeyDemand(); break;
        case Qt::Key_A: m_keys.a = down; updateKeyDemand(); break;
        case Qt::Key_D: m_keys.d = down; updateKeyDemand(); break;
        case Qt::Key_Q: m_keys.q = down; updateKeyDemand(); break;
        case Qt::Key_E: m_keys.e = down; updateKeyDemand(); break;
        case Qt::Key_R: m_keys.r = down; updateKeyDemand(); break;
        case Qt::Key_F: m_keys.f = down; updateKeyDemand(); break;

        // Toggles fire on press only, not release.
        case Qt::Key_Space: if (down) toggleArm();      break;
        case Qt::Key_1:     if (down) selectMode(0);    break;
        case Qt::Key_2:     if (down) selectMode(1);    break;
        case Qt::Key_3:     if (down) selectMode(2);    break;

        default: handled = false;
    }

    if (handled) {
        event->accept();
        return true;
    }
    return false;
}

double RovController::currentSpeed() const
{
    const double n = m_sim.currentNorth();
    const double e = m_sim.currentEast();
    return std::sqrt(n * n + e * e);
}

double RovController::currentHeading() const
{
    const double n = m_sim.currentNorth();
    const double e = m_sim.currentEast();
    if (std::fabs(n) < 1e-9 && std::fabs(e) < 1e-9)
        return 0.0;
    double deg = std::atan2(e, n) * kDegPerRad;
    if (deg < 0.0) deg += 360.0;
    return deg;
}

void RovController::setCurrent(double speed, double headingDeg, double down)
{
    const double h = headingDeg * kRadPerDeg;
    m_sim.setCurrent(speed * std::cos(h),   // north component
                     speed * std::sin(h),   // east component
                     down);
    emit telemetryChanged();
}

QString RovController::startLogging(const QString& path)
{
    QString resolved = path;
    if (resolved.isEmpty()) {
        // Default: a timestamped file in the user's home directory. No
        // directory picker needed for a one-click demo.
        const QString home =
            QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
        const QString stamp = QDateTime::currentDateTime()
                                  .toString(QStringLiteral("yyyyMMdd_hhmmss"));
        resolved = QDir(home).filePath(QStringLiteral("rov_%1.csv").arg(stamp));
    }

    const bool ok = m_logger.open(resolved.toStdString());
    if (!ok) {
        emit telemetryChanged();
        return QString();
    }

    // Reset the sample pacer so the first row lands on the next tick.
    m_nextLogAt = m_sim.state().missionTime;
    emit telemetryChanged();
    return resolved;
}

void RovController::stopLogging()
{
    m_logger.close();
    emit telemetryChanged();
}

void RovController::resetSim()
{
    m_sim.reset();
    m_keyDemand.clear();
    m_anyKeyDown = false;
    m_nextLogAt = 0.0;
    emit telemetryChanged();
}

void RovController::tick()
{
    // Measure the wall-clock time since the last tick. Qt's QTimer is subject
    // to coalescing and scheduling jitter, so the nominal 20 ms interval is a
    // request, not a guarantee. The simulator's fixed-step accumulator will
    // sub-step whatever dt we pass it.
    const qint64 nowNs = m_elapsed.nsecsElapsed();
    const double dt = double(nowNs - m_lastTickNs) * 1e-9;
    m_lastTickNs = nowNs;

    // Re-assert the keyboard demand every tick while a key is held, so the
    // dead-man timer stays reset. When the operator lets go of every key, we
    // stop pushing demand into the simulator - the timer then runs out, the
    // simulator clears its input, and thrust decays to zero through the
    // normal lag. This is what the Phase 2 tests exercise.
    if (m_anyKeyDown)
        m_sim.setControlInput(m_keyDemand);

    m_sim.step(dt);

    // Logging: rate-limit to kLogIntervalSeconds instead of writing every
    // physics tick. A fifty-row-per-second CSV is wasteful and fiddly to load.
    if (m_logger.isOpen()
        && m_sim.state().missionTime >= m_nextLogAt) {
        m_logger.write(m_sim.state().missionTime,
                       m_sim.state(),
                       m_sim.batteryCurrent(),
                       m_sim.batteryFraction());
        m_nextLogAt += kLogIntervalSeconds;
        // If the clock jumped forward (the driver was paused), snap forward
        // instead of writing a flood to catch up.
        if (m_nextLogAt < m_sim.state().missionTime)
            m_nextLogAt = m_sim.state().missionTime + kLogIntervalSeconds;
    }

    emit telemetryChanged();
}

} // namespace rov
