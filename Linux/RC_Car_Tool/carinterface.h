#ifndef CARINTERFACE_H
#define CARINTERFACE_H

#include <QWidget>
#include "datatypes.h"

namespace Ui {
class CarInterface;
}

class CarInterface : public QWidget
{
    Q_OBJECT

public:
    explicit CarInterface(QWidget *parent = 0);
    ~CarInterface();
    void setID(int id);
    int getId();
    bool pollData();
    void setOrientation(double roll, double pitch, double yaw);

private:
    Ui::CarInterface *ui;
};

#endif // CARINTERFACE_H
