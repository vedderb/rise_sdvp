/*
    Copyright 2018 Filip Franson	www.franson@hotmail.com
    Copyright 2018 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

import QtQuick 2.7
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import Car.packetInterface 1.0
import QtQuick.Controls.Styles 1.4

ApplicationWindow {
    id: minimizedWindow
    visible: true

    CustomButton {
        id: maximizeButton
        text: "MAXIMIZE"
        anchors.fill: parent
        button.onClicked: {
            applicationWindow.visible = true
        }
    }

    property PacketInterface mPacket: carClient.packetInterface()
    property double acceleration: 0.0
    property double maxAcceleration: 0.0
    property double velocity: 0.0
    property double maxVelocity: 0.0
    property int currentBatteryPercentage: 0
    property double currentBatteryVoltage: 0.0
    property bool timeRunning: true
    property double pxLast: 0.0
    property double pyLast: 0.0
    property double distanceTravelled: 0.0
    property string solutionName
    property double yaw: 0.0
    property int seconds: 0
    property int minutes: 0
    property int hours: 0
    property double startTime: 0
    property double pauseTime: 0
    property string secondsString
    property string minutesString
    property string hoursString

    ApplicationWindow {
        id: applicationWindow
        width: 600
        height: 360
        visible: true
        visibility: "FullScreen"
        color: "black"
        flags: Qt.Window | Qt.FramelessWindowHint

        onWidthChanged: {
            batteryBar.anchors.leftMargin = width * 0.1
        }

        Dialog {
            id: powerDialog
            modal: true
            visible: false
            closePolicy: Popup.CloseOnPressOutside
            height: 0
            width: applicationWindow.width/2
            x: applicationWindow.width/4
            y: applicationWindow.height/2

            onClosed: {
                powerImage.opacity = 1
                powerOptions.enabled = true
            }

            RowLayout {
                id: dialogRow
                anchors.fill: parent

                Button {
                    id: powerOffButton
                    scale: 1.4
                    opacity: 0.8
                    width: 50
                    anchors.right: cancelButton.left
                    anchors.rightMargin: 50

                    onClicked: {
                        carClient.rebootSystem(true)
                    }

                    Text {
                        id: powerText
                        text: "POWER OFF"
                        color: "black"
                        font.pointSize: 10
                        font.bold: true
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                Button {
                    id: cancelButton
                    scale: 1.4
                    opacity: 0.8
                    width: 50
                    Layout.alignment: Qt.AlignHCenter

                    onClicked: {
                        powerImage.opacity = 1
                        powerDialog.close()
                        powerOptions.enabled = true
                    }

                    Text {
                        id: cancelText
                        text: "CANCEL"
                        color: "black"
                        font.pointSize: 10
                        font.bold: true
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                Button {
                    id: rebootButton
                    scale: 1.4
                    opacity: 0.8
                    width: 50
                    anchors.left: cancelButton.right
                    anchors.leftMargin: 50

                    onClicked: {
                        carClient.rebootSystem(false)
                    }

                    Text {
                        id: rebootText
                        text: "REBOOT"
                        color: "black"
                        font.pointSize: 10
                        font.bold: true
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }
            }
        }

        Image {
            id: backgroundCar
            source: "qrc:/res/start.jpg"
            anchors.fill: parent
            fillMode: Image.PreserveAspectCrop
            opacity: 0.2
        }

        ColumnLayout {
            anchors.fill: parent

            RowLayout {
                id: topStatusBar
                Layout.fillWidth: true
                Layout.topMargin: 5

                Timer {
                    id: clockTimer
                    interval: 1000
                    repeat: true
                    running: true

                    onTriggered: {
                        clock.text = new Date().toLocaleTimeString(Qt.LocalTime)
                        var ipTxt = "IP: "
                        var addresses = carClient.getNetworkAddresses()
                        for (var i = 0;i < addresses.length;i++) {
                            ipTxt += addresses[i]
                            if (i < (addresses.length - 1)) {
                                ipTxt += " : "
                            }
                        }
                        ip.text = ipTxt
                    }
                }

                Label {
                    id: ip
                    color: "white"
                    opacity: 0.9
                    font.pointSize: 12
                    Layout.fillWidth: true
                    Layout.preferredWidth: 2000
                }

                Label {
                    id: clock
                    color: "white"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    opacity: 0.8
                    font.bold: true
                    font.pointSize: 15
                }

                Item {
                    Layout.fillWidth: true
                    Layout.preferredWidth: 2000
                    implicitHeight: clock.height

                    Rectangle {
                        id: batteryBar
                        color: "#ffffff"
                        border.color: "black"
                        border.width: 2
                        opacity: 1
                        implicitHeight: clock.height
                        anchors.fill: parent
                        anchors.leftMargin: applicationWindow.width * 0.1
                        anchors.rightMargin: applicationWindow.width * 0.1

                        Rectangle {
                            id: batteryRectangle
                            anchors.left: parent.left
                            width: Math.min(batteryBar.width * currentBatteryPercentage / 100, batteryBar.width)
                            implicitHeight: parent.height
                            opacity: 1
                            border.color: "black"
                            border.width: 2
                            color: "green"
                        }

                        Label {
                            id: batteryLabel
                            anchors.fill: parent
                            text: currentBatteryPercentage + " % (" + currentBatteryVoltage.toFixed(1) + " V)"
                            font.pointSize: batteryBar.implicitHeight/1.8
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            color: "black"
                        }
                    }
                }
            }

            SwipeView {
                id: view
                currentIndex: 0
                Layout.fillWidth: true
                Layout.fillHeight: true

                Item {
                    id: firstPage

                    ColumnLayout {
                        anchors.fill: parent

                        Item {
                            Layout.fillWidth: true
                            implicitHeight: 15
                        }

                        RowLayout {
                            id: row1
                            Layout.fillWidth: true

                            StatusDisp {
                                id: distanceDisp
                                textType: "Distance [m]"
                                textValue: "" + (distanceTravelled).toFixed(2)
                                Layout.fillWidth: true
                                implicitWidth: 2000
                                valuePointSize: 38
                            }

                            CustomButton {
                                id: startButton
                                text: "START/PAUSE"
                                Layout.fillWidth: true
                                implicitWidth: 800

                                button.onPressed: {
                                    timeRunning =! timeRunning
                                }
                            }

                            CustomButton {
                                id: resetButton
                                text: "RESET"
                                Layout.fillWidth: true
                                implicitWidth: 800

                                button.onPressed: {
                                    startTime = 0
                                    pauseTime = 0
                                    distanceTravelled = 0.0
                                    velocity = 0.0
                                    maxVelocity = 0.0
                                    maxAcceleration = 0.0
                                }
                            }

                            StatusDisp {
                                id: velocityDisp
                                textType: "Velocity [km/h]"
                                textValue: "" + (velocity*3.6).toFixed(2)
                                Layout.fillWidth: true
                                implicitWidth: 2000
                                valuePointSize: 38
                            }
                        }

                        RowLayout {
                            id: row2
                            Layout.fillWidth: true

                            Item {
                                id: filler2
                                implicitWidth: 2000
                                Layout.fillWidth: true
                                opacity: 1
                            }

                            StatusDisp {
                                id: maxAccDisp
                                textType: "max <i>a<i> [g]"
                                textValue: "" + (maxAcceleration/9.81).toFixed(2)
                                Layout.fillWidth: true
                                implicitWidth: 2000
                                valuePointSize: 35
                            }

                            StatusDisp {
                                id: maxVelDisp
                                textType: "max <i>v<i> [km/h]"
                                textValue: "" + (maxVelocity*3.6).toFixed(2)
                                Layout.fillWidth: true
                                implicitWidth: 2000
                                valuePointSize: 35
                            }

                            Item {
                                id: filler3
                                implicitWidth: 2000
                                Layout.fillWidth: true
                                opacity: 1
                            }
                        }

                        RowLayout {
                            id: row3
                            Layout.fillWidth: true

                            Item {
                                id: northArrow
                                height: 20
                                Layout.fillWidth: true
                                implicitWidth: 2000
                                Layout.bottomMargin: 140

                                Image {
                                    id: northArrowImage
                                    rotation: yaw
                                    source: "qrc:/res/north-34068.svg"
                                    anchors.horizontalCenter: parent.horizontalCenter
                                }

                                Label {
                                    id: coordinates
                                    color: "white"
                                    opacity: 0.9
                                    text: "" + pxLast.toFixed(2) + "      " + pyLast.toFixed(2)
                                    font.pointSize: 15
                                    anchors.verticalCenter: parent.verticalCenter
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    anchors.verticalCenterOffset: 180
                                    anchors.horizontalCenterOffset: 10
                                }
                            }

                            StatusDisp {
                                id: sessionTimerLabel
                                textType: "Session Time"
                                textValue: "" + hoursString + ":" + minutesString + ":" + secondsString
                                Layout.fillWidth: true
                                implicitWidth: 2000
                                valuePointSize: 30
                            }

                            StatusDisp {
                                id: nmeaStatus
                                textType: "NMEA"
                                textValue: "" + solutionName
                                Layout.fillWidth: true
                                implicitWidth: 2000
                                valuePointSize: 30
                            }
                        }
                    }
                }

                Item {
                    id: secondPage

                    ColumnLayout {
                        anchors.fill: parent

                        Item {
                            Layout.fillHeight: true
                        }

                        CustomButton {
                            id: minimizeButton
                            text: "MINIMIZE"
                            Layout.alignment: Qt.AlignHCenter
                            button.onClicked: {
                                applicationWindow.visible = false
                            }
                        }

                        Item {
                            Layout.fillHeight: true
                        }

                        MouseArea {
                            id: powerOptions
                            Layout.alignment: Qt.AlignHCenter
                            height: 150
                            width: 150

                            onPressed: {
                                powerImage.opacity = 0.5
                            }

                            onReleased: {
                                powerImage.opacity = 0
                                powerDialog.open()
                                powerOptions.enabled = false
                            }

                            Image {
                                id: powerImage
                                source: "qrc:/res/powerButton2.svg"
                                visible: true
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.horizontalCenter: parent.horizontalCenter
                                height: 125
                                width: 125
                            }
                        }

                        Item {
                            Layout.fillHeight: true
                        }
                    }
                }
            }

            PageIndicator {
                id: indicator
                Layout.alignment: Qt.AlignHCenter
                Layout.bottomMargin: 5
                count: view.count
                currentIndex: view.currentIndex
                visible: true

                delegate: Rectangle {
                    implicitHeight: 8
                    implicitWidth: 8
                    radius: 4
                    color: "white"
                    opacity: index == indicator.currentIndex ? 0.95 : pressed ? 0.7 : 0.45

                    Behavior on opacity {

                        OpacityAnimator {
                            duration: 100
                        }
                    }
                }
            }
        }

        Connections {
            property double timeLast: new Date().getTime()
            property double velLast: 0.0

            id: connections
            enabled: timeRunning
            target: mPacket

            onStateReceived: {
                stateTimer.restart()

                if ((new Date().getTime() - timeLast) < 10) {
                    return;
                }

                currentBatteryPercentage = ((state.vin - (3.4 * carClient.getBatteryCells())) /
                                            (0.8 * carClient.getBatteryCells())) * 100.0
                currentBatteryVoltage = state.vin

                if (currentBatteryPercentage < 0) {
                    currentBatteryPercentage = 0
                } else if (currentBatteryPercentage > 100) {
                    currentBatteryPercentage = 100
                }
                if (currentBatteryPercentage < 10) {
                    batteryRectangle.color = "red"
                } else if (currentBatteryPercentage < 30) {
                    batteryRectangle.color = "yellow"
                } else {
                    batteryRectangle.color = "green"
                }

                var timeDiff = (new Date().getTime() - timeLast)/1000.0
                timeLast = new Date().getTime()
                velocity = state.speed
                acceleration -= 0.05 * (acceleration - (velocity - velLast) / timeDiff)
                velLast = velocity

                if (acceleration > maxAcceleration) {
                    maxAcceleration = acceleration
                }
                if (velocity > maxVelocity) {
                    maxVelocity = velocity
                }

                var dist = Math.sqrt((state.px - pxLast) * (state.px - pxLast) +
                                     (state.py - pyLast) * (state.py - pyLast))

                if (dist < 1.5) {
                    distanceTravelled += dist
                }

                pxLast = state.px
                pyLast = state.py
                yaw = -state.yaw - 90
            }

            onNmeaRadioReceived: {
                var nmeaStr = data.toString()
                solutionName = "Unknown"
                var strs = nmeaStr.split(",")
                switch(strs[6]) {
                case "0": solutionName = "Invalid"; break
                case "1": solutionName = "SPP"; break
                case "2": solutionName = "DGPS Fix\n(" + strs[13] + " s)"; break
                case "3": solutionName = "PPS Fix"; break
                case "4": solutionName = "RTK Fix\n(" + strs[13] + " s)"; break
                case "5": solutionName = "RTK Float\n(" + strs[13] + " s)"; break
                case "6": solutionName = "Estimated (dead reckoning)"; break
                case "7": solutionName = "Manual Input Mode"; break
                case "8": solutionName = "Simulation Mode"; break
                }
            }
        }

        Timer {
            id: stateTimer
            interval: 40
            running: true
            repeat: true

            onTriggered: {
                mPacket.getState(254)
                var time = new Date().getTime()
                var timeNow = 0

                if (timeRunning == true) {
                    if (startTime == 0) {
                        startTime = time
                    }
                    timeNow = time - startTime + pauseTime
                } else if (timeRunning == false) {
                    if (startTime > 0) {
                        pauseTime = time - startTime + pauseTime
                    }
                    startTime = 0
                    timeNow = pauseTime
                }

                hours = Math.floor(timeNow/1000/3600)
                minutes = Math.floor((timeNow/1000 - hours*3600) / 60)
                seconds = timeNow/1000 - hours*3600 - minutes*60
                secondsString = seconds.toString()
                minutesString = minutes.toString()
                hoursString = hours.toString()
                if (secondsString.length < 2) {
                    secondsString = "0" + secondsString
                }
                if (minutesString.length < 2) {
                    minutesString = "0" + minutesString
                }
                if (hoursString.length < 2) {
                    hoursString = "0" + hoursString
                }
            }
        }
    }
}
