import QtQuick 2.6
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

import Car.packetInterface 1.0

ApplicationWindow {
    visible: true
    width: 400
    height: 650
    title: qsTr("Test")

    property PacketInterface mPacket: carClient.packetInterface()

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10

        CheckBox {
            id: pollBox
            text: "Poll data"
            checked: false
            Layout.preferredHeight: 48
            Layout.fillHeight: false
            Layout.fillWidth: true
        }

        Label {
            id: testLabel
            text: "Test"
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignHCenter
            Layout.bottomMargin: 5
        }

        ProgressBar {
            id: batteryBar
            Layout.preferredHeight: 48
            Layout.fillHeight: false
            Layout.fillWidth: true
            from: 0
            to: 100
            value: 10
        }

        ProgressBar {
            id: gyroZBar
            Layout.preferredHeight: 48
            Layout.fillHeight: false
            Layout.fillWidth: true
            from: -2000
            to: 2000
            value: 10
        }

        Item {
            // Spacer
            Layout.fillHeight: true
            Layout.fillWidth: true
        }
    }

    Connections {
        target: mPacket
        onStateReceived: {
            stateTimer.restart() // Only poll data if no one else does
            testLabel.text = "Input voltage " + parseFloat(state.vin).toFixed(2) + " V"
            batteryBar.value = ((state.vin - 30.0) / 12.0) * 100.0
            gyroZBar.value = state.gyro[2] * 180.0 / Math.PI
        }
    }

    Timer {
        id: stateTimer
        interval: 20
        running: pollBox.checked
        repeat: true

        onTriggered: {
            mPacket.getState(254)
        }
    }
}
