import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// ---------------------------------------------------------------------------
// Phase 0: dashboard skeleton.
//
// Every value shown here is a hard-coded placeholder. Phase 3 replaces them
// with bindings onto the C++ telemetry bridge; the layout itself is already
// the final one, so that later phases only change data flow, not structure.
// ---------------------------------------------------------------------------

ApplicationWindow {
    id: root

    width: 1360
    height: 840
    minimumWidth: 1100
    minimumHeight: 700
    visible: true
    title: qsTr("ROV Telemetry Ground Station")

    // -- Palette -------------------------------------------------------------
    readonly property color bg:        "#0e1419"
    readonly property color panel:     "#161e26"
    readonly property color panelEdge: "#24303b"
    readonly property color text:      "#e6edf3"
    readonly property color textDim:   "#7d8f9e"
    readonly property color accent:    "#2f9fd6"
    readonly property color good:      "#3fb950"
    readonly property color warn:      "#d29922"
    readonly property color bad:       "#f04747"

    color: bg

    // -- Placeholder telemetry (Phase 3 replaces with real bindings) ---------
    readonly property bool   simArmed: false
    readonly property string simMode: "MANUAL"

    // -- Reusable pieces -----------------------------------------------------

    component Panel: Rectangle {
        property string heading: ""
        default property alias content: body.data

        color: root.panel
        radius: 6
        border.width: 1
        border.color: root.panelEdge

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 14
            spacing: 10

            Label {
                text: heading
                visible: heading !== ""
                color: root.textDim
                font.pixelSize: 11
                font.letterSpacing: 1.4
                font.bold: true
            }

            ColumnLayout {
                id: body
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 8
            }
        }
    }

    component Field: RowLayout {
        property string label: ""
        property string value: "--"
        property string units: ""
        property color valueColor: root.text

        Layout.fillWidth: true
        spacing: 8

        Label {
            text: label
            color: root.textDim
            font.pixelSize: 13
            Layout.preferredWidth: 96
        }
        Label {
            text: value
            color: valueColor
            font.pixelSize: 17
            font.family: "Menlo, Consolas, monospace"
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
        }
        Label {
            text: units
            color: root.textDim
            font.pixelSize: 12
            Layout.preferredWidth: 30
        }
    }

    component Pill: Rectangle {
        property string label: ""
        property color tone: root.textDim

        implicitWidth: pillText.implicitWidth + 22
        implicitHeight: 26
        radius: 13
        color: Qt.rgba(tone.r, tone.g, tone.b, 0.15)
        border.width: 1
        border.color: tone

        Label {
            id: pillText
            anchors.centerIn: parent
            text: label
            color: tone
            font.pixelSize: 12
            font.bold: true
            font.letterSpacing: 0.8
        }
    }

    component Placeholder: Rectangle {
        property string note: ""

        Layout.fillWidth: true
        Layout.fillHeight: true
        color: "#111a21"
        radius: 4
        border.width: 1
        border.color: root.panelEdge

        Label {
            anchors.centerIn: parent
            text: note
            color: root.textDim
            font.pixelSize: 13
            horizontalAlignment: Text.AlignHCenter
        }
    }

    // -- Layout --------------------------------------------------------------

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 14
        spacing: 12

        // Header ------------------------------------------------------------
        RowLayout {
            Layout.fillWidth: true
            spacing: 14

            Label {
                text: qsTr("ROV TELEMETRY GROUND STATION")
                color: root.text
                font.pixelSize: 16
                font.bold: true
                font.letterSpacing: 1.2
            }

            Item { Layout.fillWidth: true }

            Pill {
                label: root.simArmed ? "ARMED" : "DISARMED"
                tone: root.simArmed ? root.good : root.bad
            }
            Pill {
                label: root.simMode
                tone: root.accent
            }
        }

        Rectangle {
            Layout.fillWidth: true
            implicitHeight: 1
            color: root.panelEdge
        }

        // Body ----------------------------------------------------------------
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 12

            // Left: telemetry
            ColumnLayout {
                Layout.preferredWidth: 300
                Layout.fillHeight: true
                spacing: 12

                Panel {
                    heading: "ATTITUDE"
                    Layout.fillWidth: true

                    Field { label: "Roll";  value: "0.0";  units: "deg" }
                    Field { label: "Pitch"; value: "0.0";  units: "deg" }
                    Field { label: "Yaw";   value: "0.0";  units: "deg" }
                }

                Panel {
                    heading: "POSITION"
                    Layout.fillWidth: true

                    Field { label: "Depth"; value: "0.00"; units: "m" }
                    Field { label: "North"; value: "0.0";  units: "m" }
                    Field { label: "East";  value: "0.0";  units: "m" }
                }

                Panel {
                    heading: "POWER"
                    Layout.fillWidth: true

                    Field {
                        label: "Battery"
                        value: "16.8"
                        units: "V"
                        valueColor: root.good
                    }
                    Field { label: "Mission"; value: "00:00"; units: "" }
                }

                Item { Layout.fillHeight: true }
            }

            // Middle: controls
            Panel {
                heading: "FLIGHT CONTROL"
                Layout.preferredWidth: 280
                Layout.fillHeight: true

                Button {
                    text: root.simArmed ? qsTr("DISARM") : qsTr("ARM")
                    enabled: false
                    Layout.fillWidth: true
                    Layout.preferredHeight: 44
                }

                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: 1
                    color: root.panelEdge
                }

                Label {
                    text: qsTr("MODE")
                    color: root.textDim
                    font.pixelSize: 11
                    font.letterSpacing: 1.2
                }

                Button {
                    text: qsTr("Manual")
                    enabled: false
                    Layout.fillWidth: true
                }
                Button {
                    text: qsTr("Depth Hold")
                    enabled: false
                    Layout.fillWidth: true
                }
                Button {
                    text: qsTr("Heading Hold")
                    enabled: false
                    Layout.fillWidth: true
                }

                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: 1
                    color: root.panelEdge
                }

                Label {
                    text: qsTr("TARGETS")
                    color: root.textDim
                    font.pixelSize: 11
                    font.letterSpacing: 1.2
                }

                RowLayout {
                    Layout.fillWidth: true
                    Label {
                        text: qsTr("Depth")
                        color: root.textDim
                        font.pixelSize: 13
                        Layout.fillWidth: true
                    }
                    SpinBox {
                        enabled: false
                        from: 0
                        to: 100
                        value: 0
                        Layout.preferredWidth: 120
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    Label {
                        text: qsTr("Heading")
                        color: root.textDim
                        font.pixelSize: 13
                        Layout.fillWidth: true
                    }
                    SpinBox {
                        enabled: false
                        from: 0
                        to: 359
                        value: 0
                        Layout.preferredWidth: 120
                    }
                }

                Item { Layout.fillHeight: true }
            }

            // Right: plot + map
            ColumnLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 12

                Panel {
                    heading: "DEPTH vs TIME"
                    Layout.fillWidth: true
                    Layout.fillHeight: true

                    Placeholder { note: qsTr("Depth plot — Phase 4") }
                }

                Panel {
                    heading: "POSITION — OPENSTREETMAP"
                    Layout.fillWidth: true
                    Layout.fillHeight: true

                    Placeholder { note: qsTr("OSM map view — Phase 4") }
                }
            }
        }

        // Footer ---------------------------------------------------------------
        RowLayout {
            Layout.fillWidth: true
            spacing: 10

            Label {
                text: qsTr("Phase 0 — UI skeleton. Telemetry is placeholder data.")
                color: root.textDim
                font.pixelSize: 11
            }
            Item { Layout.fillWidth: true }
            Label {
                text: Qt.application.version
                color: root.textDim
                font.pixelSize: 11
            }
        }
    }
}
