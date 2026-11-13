#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickStyle>
#include <QUrl>

int main(int argc, char* argv[])
{
    QGuiApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("ROV Telemetry Ground Station"));
    app.setApplicationVersion(QStringLiteral(ROV_APP_VERSION));
    app.setOrganizationName(QStringLiteral("Coratia Technologies"));

    // The "Basic" style gives us full control over colours on every platform.
    // The native macOS style would ignore the dark palette and render buttons
    // in system blue.
    QQuickStyle::setStyle(QStringLiteral("Basic"));

    QQmlApplicationEngine engine;

    // RovController is exposed to QML as a singleton via QML_SINGLETON in its
    // header, so no manual context-property wiring is needed here. The engine
    // instantiates it on first import.

    QObject::connect(
        &engine, &QQmlApplicationEngine::objectCreationFailed,
        &app, []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);

    engine.loadFromModule(QStringLiteral("RovGroundStation"), QStringLiteral("Main"));
    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}
