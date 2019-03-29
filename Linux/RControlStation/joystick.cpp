/*   Copyright (C) 2009  Nathaniel <linux.robotdude@gmail.com>
 *                 2012 - 2019  Benjamin Vedder <benjamin@vedder.se>
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

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <iostream>
#include "joystick.h"
#include <QMetaType>

Joystick::Joystick(QObject *parent) :
    QThread(parent)
{

    mConnected = false;
    mAxis_count = 0;
    mButton_count = 0;
    mName[0] = '\0';
    mTimer = new QTimer(this);
    mTimer->start(20);

    qRegisterMetaType<JoystickErrorType>("JoystickErrorType");

    connect(this, SIGNAL(joystickError(int,JoystickErrorType)),
            this, SLOT(errorSlot(int,JoystickErrorType)));
    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
}

Joystick::~Joystick()
{
    stop();
}

Joystick::Joystick(QString joydev, QObject *parent) :
    QThread(parent)
{
    init( joydev );
}

void Joystick::errorSlot(int error, JoystickErrorType errorType)
{
    Q_UNUSED(error);
    Q_UNUSED(errorType);
    stop();
}

void Joystick::timerSlot()
{
    for (int i = 0;i < mButtonTime.size();i++) {
        if (mButtons.at(i)) {
            mButtonTime[i]++;

            if (mButtonRepeats.at(i) && mButtonTime.at(i) > 10) {
                emit buttonChanged(i, true);
            }
        } else {
            mButtonTime[i] = 0;
        }
    }
}

int Joystick::init(QString joydev)
{
    stop();
    mConnected = false;
    mAxis_count = 0;
    mButton_count = 0;
    mJs_fd = open( joydev.toLocal8Bit().data(), O_RDONLY );
    if( mJs_fd < 0 ) {
        return -1;
    }
    mDevice = joydev;
    ioctl(mJs_fd, JSIOCGAXES, &mAxis_count);
    ioctl(mJs_fd, JSIOCGBUTTONS, &mButton_count);
    ioctl(mJs_fd, JSIOCGNAME(80), &mName);
    mAxes.resize(mAxis_count);
    mButtons.resize(mButton_count);
    mButtonRepeats.resize(mButton_count);
    mButtonTime.resize(mButton_count);
    fcntl(mJs_fd, F_SETFL, O_NONBLOCK);
    mConnected = true;

    mAbort = false;
    start(LowPriority);
    return 0;
}

void Joystick::stop()
{
    mMutex.lock();
    mAbort = true;
    mMutex.unlock();

    wait();

    mMutex.lock();
    if(mConnected) {
        close(mJs_fd);
        mConnected = false;
        mAxis_count = 0;
        mButton_count = 0;
    }
    mMutex.unlock();
}

char Joystick::getButton( int button )
{
    if(button < (mButton_count)) {
        QMutexLocker locker(&mMutex);
        return (mButtons)[button];
    }
    return -1;
}

int Joystick::getAxis( int axis )
{
    if(axis < mAxis_count) {
        QMutexLocker locker(&mMutex);
        return mAxes[axis];
    }
    return -65535;
}

QString Joystick::getName()
{
    return QString(mName);
}

QString Joystick::getDevice()
{
    return mDevice;
}

int Joystick::numButtons()
{
    return mButton_count;
}

int Joystick::numAxes()
{
    return mAxis_count;
}

bool Joystick::isConnected()
{
    return mConnected;
}

void Joystick::setRepeats(int button, bool repeats)
{
    if (button >= 0 && button < mButtonRepeats.size()) {
        mButtonRepeats[button] = repeats;
    }
}

void Joystick::run()
{
    struct js_event event;
    fd_set set;
    timeval timeout;
    int rv;

    while (false == mAbort) {
        FD_ZERO(&set);
        FD_SET(mJs_fd, &set);
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000;
        rv = select(mJs_fd + 1, &set, NULL, NULL, &timeout);

        if(rv < 0) {
            // Some error occured
            qCritical().nospace() << "Joystick select read error: " << rv;
            Q_EMIT joystickError(rv, JOYSTICK_ERROR_SELECT_READ);
        } else if(rv == 0) {
            // Timeout
        } else {
            rv = read(mJs_fd, &event, sizeof(struct js_event));
            switch ((event.type) & ~JS_EVENT_INIT) {
            case JS_EVENT_AXIS:
                mMutex.lock();
                mAxes[event.number] = event.value;
                mMutex.unlock();
                break;
            case JS_EVENT_BUTTON:
                mMutex.lock();
                mButtons[event.number] = event.value;
                mMutex.unlock();
                if (!(event.type & JS_EVENT_INIT)) {
                    Q_EMIT buttonChanged(event.number, event.value);
                }
                break;
            }

            if(rv < 0) {
                // Some error occured
                qCritical().nospace() << "Joystick read error: " << rv;
                Q_EMIT joystickError(rv, JOYSTICK_ERROR_READ);
            }
        }
    }
}
