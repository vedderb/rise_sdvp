/*
    Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#ifndef FI_H
#define FI_H

#include <QObject>
#include <QString>
#include <QStringList>

class FI : public QObject
{
    Q_OBJECT
public:
    explicit FI(QObject *parent = nullptr);

    bool fi_is_active(void);
    void fi_inject_fault_float(const char* id, float *value);
    void fi_inject_fault_int(const char* id, int *value);

    void processCmd(QString cmd);

signals:
    void sendPrint(QString s);

public slots:

private:
    // Settings
    static const int PROBE_NUM = 5;
    static const int FAULTS_PER_PROBE = 5;
    static const int FAULT_ID_MAX_LEN = 20;

    // Private types
    typedef enum {
        FAULT_TYPE_NONE = 0,
        FAULT_TYPE_BITFLIP,
        FAULT_TYPE_OFFSET,
        FAULT_TYPE_AMPLIFICATION,
        FAULT_TYPE_SET_TO
    } FAULT_TYPE_t;

    typedef struct {
        FAULT_TYPE_t type;
        float param;
        bool active;
        int start_it;
        int duration;
    } fault_info_t;

    typedef struct {
        char id[FAULT_ID_MAX_LEN + 1];
        bool active;
        fault_info_t faults[FAULTS_PER_PROBE];
        int iteration;
    } probe_t;

    // Private variables
    bool m_active = false;
    probe_t m_probes[PROBE_NUM];

    void cmd_terminal_set_enabled(int argc, const char **argv);
    void cmd_terminal_add_fault(int argc, const char **argv);
    void cmd_terminal_clear_faults(int argc, const char **argv);
    void cmd_terminal_print_faults(int argc, const char **argv);
    void cmd_terminal_reset_cnt(int argc, const char **argv);

};

#endif // FI_H
