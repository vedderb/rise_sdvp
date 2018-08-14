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

import QtQuick 2.6
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

Item {
    property alias textType: statusType.text
    property alias textValue: statusValue.text
    property alias valuePointSize: statusValue.font.pointSize
    implicitHeight: column.implicitHeight

    ColumnLayout {
        id: column
        anchors.fill: parent

        Label {
            id: statusType
            color: "white"
            opacity: 0.9
            font.underline: true
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignHCenter
        }

        Label {
            id: statusValue
            color: "white"
            opacity: 0.9
            font.pointSize: 40
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignHCenter
        }
    }
}
