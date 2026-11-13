import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import QtLocation
import QtPositioning
import RovGroundStation

// ---------------------------------------------------------------------------
// Phase 5 + attitude widget.
//
// Keyboard input is now captured by a C++ event filter installed on the
// application object, so WASD / RF / QE / Space / 1-2-3 work regardless of
// which QML widget currently has focus. The only time the filter steps aside
// is when the keyboard is being typed into a text widget (a SpinBox editor).
// ---------------------------------------------------------------------------

ApplicationWindow {
    id: root

    width: 1440
    height: 880
    minimumWidth: 1240
    minimumHeight: 760
    visible: true
    title: qsTr("ROV Telemetry Ground Station")

    // --- Palette ------------------------------------------------------------
    readonly property color bg:        "#0e1419"
    readonly property color panel:     "#161e26"
    readonly property color panelEdge: "#24303b"
    readonly property color text:      "#e6edf3"
    readonly property color textDim:   "#7d8f9e"
    readonly property color accent:    "#2f9fd6"
    readonly property color good:      "#3fb950"
    readonly property color warn:      "#d29922"
    readonly property color bad:       "#f04747"
    readonly property color plotLine:  "#4ad1e8"
    readonly property color sky:       "#245f87"
    readonly property color ground:    "#5c3a1a"

    color: bg

    // --- Plot and map buffers -----------------------------------------------
    property var  depthHistory: []
    property var  yawHistory: []
    readonly property real plotWindow: 60.0
    readonly property real depthMax:   15.0

    readonly property real originLat: 12.9716
    readonly property real originLon: 80.2594
    property var trackPath: []

    function batteryColor() {
        const f = RovController.batteryFraction
        if (f > 0.5) return good
        if (f > 0.2) return warn
        return bad
    }

    function pad2(n) { return (n < 10 ? "0" : "") + Math.floor(n) }
    function missionTimeString() {
        const t = RovController.missionTime
        return pad2(t / 60) + ":" + pad2(t % 60)
    }

    // ---- Sampling timers ---------------------------------------------------

    Timer {
        interval: 100
        running: true
        repeat: true
        onTriggered: {
            const t = RovController.missionTime
            depthHistory.push({ t: t, d: RovController.depth })
            yawHistory.push({ t: t, y: RovController.yaw })

            while (depthHistory.length > 0 && t - depthHistory[0].t > plotWindow)
                depthHistory.shift()
            while (yawHistory.length > 0 && t - yawHistory[0].t > plotWindow)
                yawHistory.shift()

            depthCanvas.requestPaint()
            headingCanvas.requestPaint()
        }
    }

    Timer {
        interval: 500
        running: true
        repeat: true
        onTriggered: {
            const dLat = RovController.north / 111320.0
            const dLon = RovController.east
                       / (111320.0 * Math.cos(originLat * Math.PI / 180.0))
            const pos = QtPositioning.coordinate(originLat + dLat,
                                                 originLon + dLon)
            const newPath = trackPath.slice()
            newPath.push(pos)
            while (newPath.length > 600)
                newPath.shift()
            trackPath = newPath
        }
    }

    // --- Attitude widget repaint every tick ---------------------------------
    Connections {
        target: RovController
        function onTelemetryChanged() {
            adi.requestPaint()
            compass.requestPaint()
            if (RovController.missionTime < 0.1 && depthHistory.length > 1) {
                depthHistory = []
                yawHistory = []
                trackPath = []
                depthCanvas.requestPaint()
                headingCanvas.requestPaint()
            }
        }
    }

    // Brief message overlay (log start/stop confirmation, errors).
    Rectangle {
        id: toast
        z: 1000
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 36
        opacity: 0
        color: Qt.rgba(root.panel.r, root.panel.g, root.panel.b, 0.95)
        border.width: 1
        border.color: root.accent
        radius: 4
        implicitWidth: toastLabel.implicitWidth + 24
        implicitHeight: toastLabel.implicitHeight + 16

        property string text: ""
        function show(msg) { text = msg; fadeIn.restart() }

        Label {
            id: toastLabel
            anchors.centerIn: parent
            color: root.text
            font.pixelSize: 12
            text: toast.text
        }

        SequentialAnimation on opacity {
            id: fadeIn
            PropertyAction { value: 1.0 }
            PauseAnimation { duration: 2800 }
            NumberAnimation { to: 0.0; duration: 600 }
        }
    }

    // -----------------------------------------------------------------------
    // Reusable inline components
    // -----------------------------------------------------------------------

    component Panel: Rectangle {
        property string heading: ""
        default property alias content: body.data

        readonly property int padding: 14
        color: root.panel
        radius: 6
        border.width: 1
        border.color: root.panelEdge

        implicitWidth: frame.implicitWidth + padding * 2
        implicitHeight: frame.implicitHeight + padding * 2

        ColumnLayout {
            id: frame
            anchors.fill: parent
            anchors.margins: parent.padding
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

    component ModeButton: Button {
        property int modeId: 0

        checkable: false
        Layout.fillWidth: true

        readonly property bool isActive: RovController.mode === modeId

        background: Rectangle {
            radius: 4
            color: parent.isActive ? Qt.rgba(root.accent.r,
                                             root.accent.g,
                                             root.accent.b, 0.20)
                                   : (parent.hovered ? "#1f2a34" : "#182028")
            border.width: 1
            border.color: parent.isActive ? root.accent : root.panelEdge
        }
        contentItem: Label {
            text: parent.text
            color: parent.isActive ? root.accent : root.text
            font.pixelSize: 13
            font.bold: parent.isActive
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }

        onClicked: RovController.selectMode(modeId)
    }

    component ThrustBar: Item {
        property string label: ""
        property real value: 0.0

        implicitHeight: 18
        Layout.fillWidth: true

        RowLayout {
            anchors.fill: parent
            spacing: 8

            Label {
                text: parent.parent.label
                color: root.textDim
                font.pixelSize: 11
                Layout.preferredWidth: 46
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 10
                color: "#0b1116"
                radius: 2
                border.width: 1
                border.color: root.panelEdge

                Rectangle {
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    width: 1
                    height: parent.height
                    color: root.panelEdge
                }

                Rectangle {
                    x: parent.width / 2 + (parent.parent.parent.value < 0
                                           ? parent.width / 2 * parent.parent.parent.value
                                           : 0)
                    width: Math.abs(parent.parent.parent.value) * parent.width / 2
                    height: parent.height - 2
                    anchors.verticalCenter: parent.verticalCenter
                    color: parent.parent.parent.value >= 0 ? root.accent : root.warn
                    radius: 2
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Dashboard layout
    // -----------------------------------------------------------------------

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 14
        spacing: 12

        // Header ---------------------------------------------------------
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
                label: RovController.armed ? "ARMED" : "DISARMED"
                tone: RovController.armed ? root.good : root.bad
            }
            Pill {
                label: RovController.modeName
                tone: root.accent
            }
        }

        Rectangle {
            Layout.fillWidth: true
            implicitHeight: 1
            color: root.panelEdge
        }

        // Body -----------------------------------------------------------
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 12

            // --- Left column: thin telemetry ----------------------------
            ColumnLayout {
                Layout.preferredWidth: 260
                Layout.fillHeight: true
                spacing: 12

                Panel {
                    heading: "POSITION"
                    Layout.fillWidth: true

                    Field { label: "Depth"; value: RovController.depth.toFixed(2); units: "m" }
                    Field { label: "North"; value: RovController.north.toFixed(1); units: "m" }
                    Field { label: "East";  value: RovController.east.toFixed(1);  units: "m" }
                }

                Panel {
                    heading: "POWER"
                    Layout.fillWidth: true

                    Field {
                        label: "Battery"
                        value: RovController.batteryVolts.toFixed(1)
                        units: "V"
                        valueColor: root.batteryColor()
                    }
                    Field {
                        label: "Current"
                        value: RovController.batteryCurrent.toFixed(1)
                        units: "A"
                    }
                    Field {
                        label: "Mission"
                        value: root.missionTimeString()
                    }
                }

                Panel {
                    heading: "THRUST"
                    Layout.fillWidth: true

                    ThrustBar { label: "Surge"; value: RovController.surgeThrust }
                    ThrustBar { label: "Sway";  value: RovController.swayThrust }
                    ThrustBar { label: "Heave"; value: RovController.heaveThrust }
                    ThrustBar { label: "Yaw";   value: RovController.yawThrust }
                }

                Panel {
                    heading: "ENVIRONMENT"
                    Layout.fillWidth: true

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        Label {
                            text: qsTr("Current")
                            color: root.textDim
                            font.pixelSize: 13
                            Layout.fillWidth: true
                        }
                        SpinBox {
                            id: currentSpeedSpin
                            from: 0
                            to: 100
                            stepSize: 5
                            value: Math.round(RovController.currentSpeed * 10)
                            editable: true
                            Layout.preferredWidth: 90

                            textFromValue: function(value) { return (value / 10).toFixed(1) + " m/s" }
                            valueFromText: function(text)  { return Math.round(parseFloat(text) * 10) }

                            onValueModified: RovController.setCurrent(
                                value / 10,
                                currentHeadingSpin.value,
                                RovController.currentDown)
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        Label {
                            text: qsTr("Heading")
                            color: root.textDim
                            font.pixelSize: 13
                            Layout.fillWidth: true
                        }
                        SpinBox {
                            id: currentHeadingSpin
                            from: 0
                            to: 359
                            value: Math.round(RovController.currentHeading)
                            editable: true
                            Layout.preferredWidth: 90

                            textFromValue: function(value) { return value + "°" }

                            onValueModified: RovController.setCurrent(
                                currentSpeedSpin.value / 10,
                                value,
                                RovController.currentDown)
                        }
                    }
                }

                Item { Layout.fillHeight: true }
            }

            // --- Center column: ATTITUDE WIDGET -------------------------
            Panel {
                id: attitudePanel
                heading: "ATTITUDE"
                Layout.preferredWidth: 320
                Layout.fillHeight: true

                // Artificial horizon ------------------------------------
                Item {
                    id: adiHolder
                    Layout.fillWidth: true
                    Layout.preferredHeight: 240

                    Canvas {
                        id: adi
                        anchors.fill: parent

                        onWidthChanged: requestPaint()
                        onHeightChanged: requestPaint()

                        onPaint: {
                            const ctx = getContext("2d")
                            ctx.reset()

                            const cx = width / 2
                            const cy = height / 2
                            const r  = Math.min(width, height) / 2 - 4
                            if (r < 20) return

                            const roll  = RovController.roll    // deg
                            const pitch = RovController.pitch   // deg (nose-up +)
                            const pxPerDeg = r / 45.0           // ±45° fits in frame

                            ctx.save()

                            // Circular clip for the horizon world.
                            ctx.beginPath()
                            ctx.arc(cx, cy, r, 0, Math.PI * 2)
                            ctx.clip()

                            // Transform into attitude frame: rotate by -roll,
                            // translate by pitch (nose up = horizon down).
                            ctx.translate(cx, cy)
                            ctx.rotate(-roll * Math.PI / 180)
                            ctx.translate(0, pitch * pxPerDeg)

                            // Sky and ground, large enough to cover the full
                            // disk at any rotation.
                            const big = r * 3
                            ctx.fillStyle = root.sky
                            ctx.fillRect(-big, -big, big * 2, big)
                            ctx.fillStyle = root.ground
                            ctx.fillRect(-big, 0, big * 2, big)

                            // Horizon line.
                            ctx.strokeStyle = "#ffffff"
                            ctx.lineWidth = 2
                            ctx.beginPath()
                            ctx.moveTo(-big, 0)
                            ctx.lineTo(big, 0)
                            ctx.stroke()

                            // Pitch ladder: ticks every 10°, major at 30°.
                            ctx.lineWidth = 1
                            ctx.font = "9px Menlo, Consolas, monospace"
                            ctx.fillStyle = "#ffffff"
                            ctx.textAlign = "right"
                            ctx.textBaseline = "middle"
                            for (let p = -30; p <= 30; p += 10) {
                                if (p === 0) continue
                                const y = p * pxPerDeg
                                const half = (Math.abs(p) === 30) ? 30 : 20
                                ctx.beginPath()
                                ctx.moveTo(-half, y)
                                ctx.lineTo(half, y)
                                ctx.stroke()
                                ctx.fillText((-p).toString(), -half - 4, y)
                            }

                            ctx.restore()

                            // Fixed overlay: bezel.
                            ctx.strokeStyle = root.panelEdge
                            ctx.lineWidth = 2
                            ctx.beginPath()
                            ctx.arc(cx, cy, r, 0, Math.PI * 2)
                            ctx.stroke()

                            // Roll scale on top of bezel.
                            ctx.strokeStyle = root.textDim
                            ctx.lineWidth = 1
                            for (let a = -60; a <= 60; a += 10) {
                                const rad = (a - 90) * Math.PI / 180
                                const x1 = cx + Math.cos(rad) * r
                                const y1 = cy + Math.sin(rad) * r
                                const tickLen = (a % 30 === 0) ? 8 : 4
                                const x2 = cx + Math.cos(rad) * (r - tickLen)
                                const y2 = cy + Math.sin(rad) * (r - tickLen)
                                ctx.beginPath()
                                ctx.moveTo(x1, y1)
                                ctx.lineTo(x2, y2)
                                ctx.stroke()
                            }
                            // Fixed roll index (downward triangle) at top.
                            ctx.fillStyle = root.text
                            ctx.beginPath()
                            ctx.moveTo(cx, cy - r + 10)
                            ctx.lineTo(cx - 5, cy - r)
                            ctx.lineTo(cx + 5, cy - r)
                            ctx.closePath()
                            ctx.fill()

                            // Rotating roll pointer (short mark on the rim).
                            ctx.save()
                            ctx.translate(cx, cy)
                            ctx.rotate(-roll * Math.PI / 180)
                            ctx.fillStyle = root.warn
                            ctx.beginPath()
                            ctx.moveTo(0, -r + 10)
                            ctx.lineTo(-4, -r + 2)
                            ctx.lineTo(4, -r + 2)
                            ctx.closePath()
                            ctx.fill()
                            ctx.restore()

                            // Fixed aircraft reference: a yellow W-shape.
                            ctx.strokeStyle = "#f0c419"
                            ctx.lineWidth = 3
                            ctx.lineCap = "round"
                            ctx.beginPath()
                            ctx.moveTo(cx - 30, cy)
                            ctx.lineTo(cx - 10, cy)
                            ctx.lineTo(cx - 5,  cy + 6)
                            ctx.moveTo(cx + 30, cy)
                            ctx.lineTo(cx + 10, cy)
                            ctx.lineTo(cx + 5,  cy + 6)
                            ctx.stroke()
                            ctx.fillStyle = "#f0c419"
                            ctx.beginPath()
                            ctx.arc(cx, cy, 2.5, 0, Math.PI * 2)
                            ctx.fill()
                        }
                    }
                }

                // Compass / HSI ---------------------------------------
                Item {
                    id: compassHolder
                    Layout.fillWidth: true
                    Layout.preferredHeight: 160

                    Canvas {
                        id: compass
                        anchors.fill: parent

                        onWidthChanged: requestPaint()
                        onHeightChanged: requestPaint()

                        onPaint: {
                            const ctx = getContext("2d")
                            ctx.reset()

                            const cx = width / 2
                            const cy = height / 2
                            const r  = Math.min(width, height) / 2 - 4
                            if (r < 20) return

                            const yaw    = RovController.yaw
                            const target = RovController.targetHeading
                            const inHdg  = (RovController.mode === 2)

                            // Bezel.
                            ctx.fillStyle = "#0b1116"
                            ctx.beginPath()
                            ctx.arc(cx, cy, r, 0, Math.PI * 2)
                            ctx.fill()
                            ctx.strokeStyle = root.panelEdge
                            ctx.lineWidth = 2
                            ctx.stroke()

                            // Rotating compass card.
                            ctx.save()
                            ctx.translate(cx, cy)
                            ctx.rotate(-yaw * Math.PI / 180)

                            ctx.strokeStyle = root.textDim
                            ctx.lineWidth = 1
                            ctx.font = "10px Menlo, Consolas, monospace"
                            ctx.fillStyle = root.text
                            ctx.textAlign = "center"
                            ctx.textBaseline = "middle"

                            for (let a = 0; a < 360; a += 10) {
                                const rad = (a - 90) * Math.PI / 180
                                const major = (a % 30 === 0)
                                const tickLen = major ? 10 : 4
                                const x1 = Math.cos(rad) * r
                                const y1 = Math.sin(rad) * r
                                const x2 = Math.cos(rad) * (r - tickLen)
                                const y2 = Math.sin(rad) * (r - tickLen)
                                ctx.beginPath()
                                ctx.moveTo(x1, y1)
                                ctx.lineTo(x2, y2)
                                ctx.stroke()
                            }

                            // Cardinal letters.
                            ctx.font = "bold 13px sans-serif"
                            const labels = [
                                { a:   0, s: "N", c: root.bad },
                                { a:  90, s: "E", c: root.text },
                                { a: 180, s: "S", c: root.text },
                                { a: 270, s: "W", c: root.text },
                            ]
                            for (const L of labels) {
                                ctx.save()
                                ctx.rotate(L.a * Math.PI / 180)
                                ctx.fillStyle = L.c
                                ctx.fillText(L.s, 0, -r + 20)
                                ctx.restore()
                            }

                            // Target heading bug (orange triangle) on the rim,
                            // visible when Heading Hold is active.
                            if (inHdg) {
                                ctx.save()
                                ctx.rotate(target * Math.PI / 180)
                                ctx.fillStyle = root.warn
                                ctx.beginPath()
                                ctx.moveTo(0, -r)
                                ctx.lineTo(-5, -r + 10)
                                ctx.lineTo(5,  -r + 10)
                                ctx.closePath()
                                ctx.fill()
                                ctx.restore()
                            }

                            ctx.restore()

                            // Fixed lubber line - a notch at the top showing
                            // where the current heading sits on the card.
                            ctx.fillStyle = root.plotLine
                            ctx.beginPath()
                            ctx.moveTo(cx, cy - r - 2)
                            ctx.lineTo(cx - 6, cy - r - 12)
                            ctx.lineTo(cx + 6, cy - r - 12)
                            ctx.closePath()
                            ctx.fill()

                            // Centre dot.
                            ctx.fillStyle = root.textDim
                            ctx.beginPath()
                            ctx.arc(cx, cy, 2.5, 0, Math.PI * 2)
                            ctx.fill()
                        }
                    }
                }

                // Numeric readout -------------------------------------
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10

                    ColumnLayout {
                        spacing: 0
                        Layout.fillWidth: true
                        Label {
                            text: "ROLL"
                            color: root.textDim
                            font.pixelSize: 10
                            font.letterSpacing: 1.2
                        }
                        Label {
                            text: RovController.roll.toFixed(1) + "°"
                            color: root.text
                            font.pixelSize: 16
                            font.family: "Menlo, Consolas, monospace"
                        }
                    }
                    ColumnLayout {
                        spacing: 0
                        Layout.fillWidth: true
                        Label {
                            text: "PITCH"
                            color: root.textDim
                            font.pixelSize: 10
                            font.letterSpacing: 1.2
                        }
                        Label {
                            text: RovController.pitch.toFixed(1) + "°"
                            color: root.text
                            font.pixelSize: 16
                            font.family: "Menlo, Consolas, monospace"
                        }
                    }
                    ColumnLayout {
                        spacing: 0
                        Layout.fillWidth: true
                        Label {
                            text: "HEADING"
                            color: root.textDim
                            font.pixelSize: 10
                            font.letterSpacing: 1.2
                        }
                        Label {
                            text: RovController.yaw.toFixed(1) + "°"
                            color: root.text
                            font.pixelSize: 16
                            font.family: "Menlo, Consolas, monospace"
                        }
                    }
                }

                Item { Layout.fillHeight: true }
            }

            // --- Flight Control -----------------------------------------
            Panel {
                heading: "FLIGHT CONTROL"
                Layout.preferredWidth: 280
                Layout.fillHeight: true

                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 48
                    text: RovController.armed ? qsTr("DISARM") : qsTr("ARM")

                    background: Rectangle {
                        radius: 4
                        color: RovController.armed
                               ? Qt.rgba(root.bad.r, root.bad.g, root.bad.b, 0.20)
                               : Qt.rgba(root.good.r, root.good.g, root.good.b, 0.20)
                        border.width: 1
                        border.color: RovController.armed ? root.bad : root.good
                    }
                    contentItem: Label {
                        text: parent.text
                        color: RovController.armed ? root.bad : root.good
                        font.pixelSize: 15
                        font.bold: true
                        font.letterSpacing: 1.2
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }

                    onClicked: RovController.toggleArm()
                }

                Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: root.panelEdge }

                Label {
                    text: qsTr("MODE")
                    color: root.textDim
                    font.pixelSize: 11
                    font.letterSpacing: 1.2
                }

                ModeButton { text: qsTr("Manual");       modeId: 0 }
                ModeButton { text: qsTr("Depth Hold");   modeId: 1 }
                ModeButton { text: qsTr("Heading Hold"); modeId: 2 }

                Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: root.panelEdge }

                Label {
                    text: qsTr("TARGETS")
                    color: root.textDim
                    font.pixelSize: 11
                    font.letterSpacing: 1.2
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    Label {
                        text: qsTr("Depth")
                        color: root.textDim
                        font.pixelSize: 13
                        Layout.fillWidth: true
                    }
                    SpinBox {
                        from: 0
                        to: 100
                        value: Math.round(RovController.targetDepth)
                        editable: true
                        Layout.preferredWidth: 110
                        onValueModified: RovController.setTargetDepth(value)
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    Label {
                        text: qsTr("Heading")
                        color: root.textDim
                        font.pixelSize: 13
                        Layout.fillWidth: true
                    }
                    SpinBox {
                        from: 0
                        to: 359
                        value: Math.round(RovController.targetHeading)
                        editable: true
                        Layout.preferredWidth: 110
                        onValueModified: RovController.setTargetHeading(value)
                    }
                }

                Item { Layout.fillHeight: true; implicitHeight: 1 }

                Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: root.panelEdge }

                Button {
                    Layout.fillWidth: true
                    text: RovController.logging
                          ? qsTr("■  Stop Logging")
                          : qsTr("●  Start Logging")

                    background: Rectangle {
                        radius: 4
                        color: RovController.logging
                               ? Qt.rgba(root.warn.r, root.warn.g, root.warn.b, 0.18)
                               : (parent.hovered ? "#1f2a34" : "#182028")
                        border.width: 1
                        border.color: RovController.logging ? root.warn : root.panelEdge
                    }
                    contentItem: Label {
                        text: parent.text
                        color: RovController.logging ? root.warn : root.text
                        font.pixelSize: 12
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }

                    onClicked: {
                        if (RovController.logging) {
                            RovController.stopLogging()
                        } else {
                            const where = RovController.startLogging("")
                            if (where.length > 0)
                                toast.show(qsTr("Logging to %1").arg(where))
                            else
                                toast.show(qsTr("Could not open log file"))
                        }
                    }
                }

                Label {
                    visible: RovController.logging
                    text: qsTr("%1 rows").arg(RovController.logRows)
                    color: root.textDim
                    font.pixelSize: 11
                    font.family: "Menlo, Consolas, monospace"
                    Layout.alignment: Qt.AlignRight
                }

                Button {
                    Layout.fillWidth: true
                    text: qsTr("Reset Simulation")

                    background: Rectangle {
                        radius: 4
                        color: parent.hovered ? "#1f2a34" : "#182028"
                        border.width: 1
                        border.color: root.panelEdge
                    }
                    contentItem: Label {
                        text: parent.text
                        color: root.textDim
                        font.pixelSize: 12
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }

                    onClicked: RovController.resetSim()
                }
            }

            // --- Right column: depth plot, heading plot, map -----------
            ColumnLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 12

                Panel {
                    heading: "DEPTH vs TIME"
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredHeight: 1   // equal weight share

                    Canvas {
                        id: depthCanvas
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        implicitHeight: 160

                        onWidthChanged:  requestPaint()
                        onHeightChanged: requestPaint()

                        onPaint: {
                            const ctx = getContext("2d")
                            ctx.reset()

                            const margin = { l: 42, r: 10, t: 8, b: 20 }
                            const plotW = width  - margin.l - margin.r
                            const plotH = height - margin.t - margin.b
                            if (plotW < 20 || plotH < 20) return

                            const now = RovController.missionTime
                            const windowSec = root.plotWindow
                            const tMin = Math.max(0, now - windowSec)

                            let yMax = root.depthMax
                            for (let i = 0; i < root.depthHistory.length; ++i)
                                yMax = Math.max(yMax, root.depthHistory[i].d + 1)
                            yMax = Math.ceil(yMax / 5) * 5

                            ctx.fillStyle = "#0b1116"
                            ctx.fillRect(margin.l, margin.t, plotW, plotH)
                            ctx.strokeStyle = root.panelEdge
                            ctx.lineWidth = 1
                            ctx.strokeRect(margin.l + 0.5, margin.t + 0.5,
                                           plotW - 1, plotH - 1)

                            function tx(t) { return margin.l + plotW * (t - tMin) / windowSec }
                            function dy(d) { return margin.t + plotH * d / yMax }

                            const dStep = yMax <= 10 ? 2 : (yMax <= 20 ? 4 : 10)
                            ctx.strokeStyle = "#1a2530"
                            ctx.fillStyle = root.textDim
                            ctx.font = "10px Menlo, Consolas, monospace"
                            ctx.textAlign = "right"
                            ctx.textBaseline = "middle"
                            for (let d = 0; d <= yMax; d += dStep) {
                                const y = dy(d)
                                ctx.beginPath()
                                ctx.moveTo(margin.l, y + 0.5)
                                ctx.lineTo(margin.l + plotW, y + 0.5)
                                ctx.stroke()
                                ctx.fillText(d.toFixed(0) + " m", margin.l - 4, y)
                            }

                            ctx.textAlign = "center"
                            ctx.textBaseline = "top"
                            for (let t = Math.ceil(tMin / 10) * 10; t <= now + 1e-6; t += 10) {
                                const x = tx(t)
                                ctx.beginPath()
                                ctx.moveTo(x + 0.5, margin.t)
                                ctx.lineTo(x + 0.5, margin.t + plotH)
                                ctx.stroke()
                                ctx.fillText(t.toFixed(0) + "s", x, margin.t + plotH + 2)
                            }

                            if (RovController.mode === 1) {
                                const yt = dy(RovController.targetDepth)
                                ctx.strokeStyle = root.warn
                                ctx.setLineDash([4, 4])
                                ctx.lineWidth = 1
                                ctx.beginPath()
                                ctx.moveTo(margin.l, yt + 0.5)
                                ctx.lineTo(margin.l + plotW, yt + 0.5)
                                ctx.stroke()
                                ctx.setLineDash([])
                            }

                            const n = root.depthHistory.length
                            if (n >= 2) {
                                ctx.strokeStyle = root.plotLine
                                ctx.lineWidth = 2
                                ctx.lineJoin = "round"
                                ctx.beginPath()
                                let started = false
                                for (let i = 0; i < n; ++i) {
                                    const s = root.depthHistory[i]
                                    if (s.t < tMin) continue
                                    const x = tx(s.t)
                                    const y = dy(s.d)
                                    if (!started) { ctx.moveTo(x, y); started = true }
                                    else          { ctx.lineTo(x, y) }
                                }
                                ctx.stroke()
                            }
                        }
                    }
                }

                Panel {
                    heading: "HEADING vs TIME"
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredHeight: 1

                    Canvas {
                        id: headingCanvas
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        implicitHeight: 160

                        onWidthChanged:  requestPaint()
                        onHeightChanged: requestPaint()

                        onPaint: {
                            const ctx = getContext("2d")
                            ctx.reset()

                            const margin = { l: 42, r: 10, t: 8, b: 20 }
                            const plotW = width  - margin.l - margin.r
                            const plotH = height - margin.t - margin.b
                            if (plotW < 20 || plotH < 20) return

                            const now = RovController.missionTime
                            const windowSec = root.plotWindow
                            const tMin = Math.max(0, now - windowSec)

                            ctx.fillStyle = "#0b1116"
                            ctx.fillRect(margin.l, margin.t, plotW, plotH)
                            ctx.strokeStyle = root.panelEdge
                            ctx.lineWidth = 1
                            ctx.strokeRect(margin.l + 0.5, margin.t + 0.5,
                                           plotW - 1, plotH - 1)

                            function tx(t) { return margin.l + plotW * (t - tMin) / windowSec }
                            function yy(h) { return margin.t + plotH * h / 360.0 }

                            ctx.strokeStyle = "#1a2530"
                            ctx.fillStyle = root.textDim
                            ctx.font = "10px Menlo, Consolas, monospace"
                            ctx.textAlign = "right"
                            ctx.textBaseline = "middle"
                            for (let h = 0; h <= 360; h += 90) {
                                const y = yy(h)
                                ctx.beginPath()
                                ctx.moveTo(margin.l, y + 0.5)
                                ctx.lineTo(margin.l + plotW, y + 0.5)
                                ctx.stroke()
                                ctx.fillText(h.toFixed(0) + "°", margin.l - 4, y)
                            }

                            ctx.textAlign = "center"
                            ctx.textBaseline = "top"
                            for (let t = Math.ceil(tMin / 10) * 10; t <= now + 1e-6; t += 10) {
                                const x = tx(t)
                                ctx.beginPath()
                                ctx.moveTo(x + 0.5, margin.t)
                                ctx.lineTo(x + 0.5, margin.t + plotH)
                                ctx.stroke()
                                ctx.fillText(t.toFixed(0) + "s", x, margin.t + plotH + 2)
                            }

                            // Target heading in Heading Hold.
                            if (RovController.mode === 2) {
                                const yt = yy(RovController.targetHeading)
                                ctx.strokeStyle = root.warn
                                ctx.setLineDash([4, 4])
                                ctx.lineWidth = 1
                                ctx.beginPath()
                                ctx.moveTo(margin.l, yt + 0.5)
                                ctx.lineTo(margin.l + plotW, yt + 0.5)
                                ctx.stroke()
                                ctx.setLineDash([])
                            }

                            // Heading trace, with pen-lift at the 360->0 wrap so
                            // the line doesn't draw a vertical streak across the
                            // whole plot every time the vehicle crosses north.
                            const n = root.yawHistory.length
                            if (n >= 2) {
                                ctx.strokeStyle = root.plotLine
                                ctx.lineWidth = 2
                                ctx.lineJoin = "round"
                                let drawing = false
                                let prevY = 0
                                ctx.beginPath()
                                for (let i = 0; i < n; ++i) {
                                    const s = root.yawHistory[i]
                                    if (s.t < tMin) continue
                                    const x = tx(s.t)
                                    const y = yy(s.y)
                                    if (!drawing) {
                                        ctx.moveTo(x, y)
                                        drawing = true
                                    } else {
                                        // If adjacent samples jumped more than
                                        // half the range, it was a wrap - lift
                                        // the pen and start a new segment.
                                        if (Math.abs(s.y - root.yawHistory[i - 1].y) > 180.0) {
                                            ctx.stroke()
                                            ctx.beginPath()
                                            ctx.moveTo(x, y)
                                        } else {
                                            ctx.lineTo(x, y)
                                        }
                                    }
                                    prevY = y
                                }
                                ctx.stroke()
                            }
                        }
                    }
                }

                Panel {
                    heading: "POSITION — OPENSTREETMAP"
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredHeight: 2   // give the map more room than a plot

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        implicitHeight: 220
                        color: "#0b1116"
                        radius: 4
                        border.width: 1
                        border.color: root.panelEdge
                        clip: true

                        Map {
                            id: mapView
                            anchors.fill: parent
                            anchors.margins: 1

                            plugin: Plugin { name: "osm" }
                            center: trackPath.length > 0
                                    ? trackPath[trackPath.length - 1]
                                    : QtPositioning.coordinate(root.originLat,
                                                               root.originLon)
                            zoomLevel: 18
                            copyrightsVisible: false

                            MapPolyline {
                                line.width: 3
                                line.color: root.plotLine
                                path: root.trackPath
                            }

                            MapQuickItem {
                                coordinate: QtPositioning.coordinate(root.originLat,
                                                                     root.originLon)
                                anchorPoint.x: 5
                                anchorPoint.y: 5
                                sourceItem: Rectangle {
                                    width: 10; height: 10; radius: 2
                                    color: "transparent"
                                    border.color: root.textDim
                                    border.width: 2
                                }
                            }

                            MapQuickItem {
                                visible: RovController.currentSpeed > 0.01
                                coordinate: QtPositioning.coordinate(root.originLat,
                                                                     root.originLon)
                                anchorPoint.x: 2
                                anchorPoint.y: 42
                                sourceItem: Item {
                                    width: 4
                                    height: 40
                                    transformOrigin: Item.Bottom
                                    rotation: RovController.currentHeading
                                    Rectangle {
                                        anchors.fill: parent
                                        anchors.bottomMargin: 6
                                        color: root.warn
                                        opacity: 0.85
                                    }
                                    Canvas {
                                        width: 14; height: 10
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        anchors.top: parent.top
                                        anchors.topMargin: -4
                                        onPaint: {
                                            const ctx = getContext("2d")
                                            ctx.reset()
                                            ctx.fillStyle = root.warn
                                            ctx.beginPath()
                                            ctx.moveTo(width / 2, 0)
                                            ctx.lineTo(width, height)
                                            ctx.lineTo(0, height)
                                            ctx.closePath()
                                            ctx.fill()
                                        }
                                    }
                                }
                            }

                            MapQuickItem {
                                coordinate: trackPath.length > 0
                                            ? trackPath[trackPath.length - 1]
                                            : QtPositioning.coordinate(root.originLat,
                                                                       root.originLon)
                                anchorPoint.x: 9
                                anchorPoint.y: 9
                                sourceItem: Item {
                                    width: 18
                                    height: 18
                                    Rectangle {
                                        anchors.fill: parent
                                        radius: width / 2
                                        color: Qt.rgba(root.plotLine.r,
                                                       root.plotLine.g,
                                                       root.plotLine.b, 0.25)
                                    }
                                    Rectangle {
                                        width: 8
                                        height: 8
                                        radius: 4
                                        anchors.centerIn: parent
                                        color: root.plotLine
                                        border.color: "white"
                                        border.width: 1
                                    }
                                    Rectangle {
                                        width: 2
                                        height: 10
                                        color: root.plotLine
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        anchors.bottom: parent.verticalCenter
                                        transformOrigin: Item.Bottom
                                        rotation: RovController.yaw
                                    }
                                }
                            }
                        }

                        Rectangle {
                            anchors.bottom: parent.bottom
                            anchors.right: parent.right
                            anchors.margins: 6
                            width: legendText.implicitWidth + 12
                            height: legendText.implicitHeight + 6
                            color: Qt.rgba(root.panel.r,
                                           root.panel.g,
                                           root.panel.b, 0.85)
                            radius: 3
                            border.width: 1
                            border.color: root.panelEdge
                            Label {
                                id: legendText
                                anchors.centerIn: parent
                                color: root.textDim
                                font.pixelSize: 10
                                text: qsTr("© OpenStreetMap  ·  origin %1,%2")
                                      .arg(root.originLat.toFixed(4))
                                      .arg(root.originLon.toFixed(4))
                            }
                        }
                    }
                }
            }
        }

        // Footer -------------------------------------------------------
        RowLayout {
            Layout.fillWidth: true
            spacing: 16

            Label {
                text: qsTr("WASD move · QE yaw · RF dive/rise · Space arm · 1/2/3 mode")
                color: root.textDim
                font.pixelSize: 11
            }

            Item { Layout.fillWidth: true }

            Label {
                text: qsTr("Dead-man:  %1 s")
                      .arg(RovController.timeSinceInput.toFixed(1))
                color: RovController.timeSinceInput > 1.0
                       ? root.warn : root.textDim
                font.pixelSize: 11
                font.family: "Menlo, Consolas, monospace"
            }

            Label {
                text: Qt.application.version
                color: root.textDim
                font.pixelSize: 11
            }
        }
    }
}
