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

    // "Basic" gives us full control over colours on every platform. The native
    // macOS style would override the dark palette the dashboard depends on.
    QQuickStyle::setStyle(QStringLiteral("Basic"));

    QQmlApplicationEngine engine;

    QObject::connect(
        &engine, &QQmlApplicationEngine::objectCreationFailed,
        &app, []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);

    engine.load(QUrl(QStringLiteral("qrc:/qt/qml/RovGroundStation/qml/Main.qml")));
    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}
