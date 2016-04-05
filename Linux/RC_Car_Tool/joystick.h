/*   Copyright (C) 2009  Nathaniel <linux.robotdude@gmail.com>
 *                 2012  Benjamin Vedder <benjamin@vedder.se>
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include <QMutex>
#include <QSize>
#include <QThread>
#include <QWaitCondition>
#include <QtDebug>

#ifdef UNIX
#include <linux/joystick.h>
#endif

enum JoystickErrorType{JOYSTICK_ERROR_READ,
		       JOYSTICK_ERROR_WRITE,
		       JOYSTICK_ERROR_SELECT_READ,
		       JOYSTICK_ERROR_SELECT_WRITE};

class Joystick : public QThread
{
    Q_OBJECT

public:
    Joystick(QObject *parent = 0);
    ~Joystick();
    Joystick(QString joydev, QObject *parent = 0);
    int init(QString joydev);
    void stop();
    char getButton( int button );
    int getAxis( int axis );
    QString getName();
    QString getDevice();
    int numAxes();
    int numButtons();
    bool isConnected();

Q_SIGNALS:
    void buttonPressed(int button);
    void joystickError(int error, JoystickErrorType errorType);

public Q_SLOTS:
    void errorSlot(int error, JoystickErrorType errorType);

protected:
    void run();

    QMutex mMutex;
    bool mAbort;

    QString mDevice;
    int mJs_fd;
    int mAxis_count;
    int mButton_count;
    char mName[80];
    int *mAxes;
    char *mButtons;
    bool mConnected;
};

#endif
