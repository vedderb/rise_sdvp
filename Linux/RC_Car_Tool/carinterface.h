#ifndef CARINTERFACE_H
#define CARINTERFACE_H

#include <QWidget>

namespace Ui {
class CarInterface;
}

class CarInterface : public QWidget
{
    Q_OBJECT

public:
    explicit CarInterface(QWidget *parent = 0);
    void setID(int id);
    int getId();
    ~CarInterface();

private:
    Ui::CarInterface *ui;
};

#endif // CARINTERFACE_H
