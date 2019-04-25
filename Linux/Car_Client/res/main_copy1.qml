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
    id: applicationWindow
    width: 800
    height: 480
    visible: true
    visibility: "FullScreen"
    color: "black"
    flags: Qt.Window | Qt.FramelessWindowHint

    property PacketInterface mPacket: carClient.packetInterface()

    Label {
        text: "LABEL!"
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

        SwipeView {
            id: view
            currentIndex: 0
            Layout.fillWidth: true
            Layout.fillHeight: true

            Item {
                id: secondPage

                ColumnLayout {
                    anchors.fill: parent

                    Item {
                        Layout.fillHeight: true
                    }

                    CustomButton {
                        id: minimizeButton
                        text: "CLOSE"

                        Layout.alignment: Qt.AlignHCenter
                        button.onClicked: {
                            Qt.quit()
                            // applicationWindow.visible = false
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
    }
}
