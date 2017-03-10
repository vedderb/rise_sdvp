#ifndef CONFCOMMONWIDGET_H
#define CONFCOMMONWIDGET_H

#include <QWidget>
#include <QVector>
#include "datatypes.h"

namespace Ui {
class ConfCommonWidget;
}

class ConfCommonWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ConfCommonWidget(QWidget *parent = 0);
    ~ConfCommonWidget();

    void getConfGui(MAIN_CONFIG &conf);
    void setConfGui(const MAIN_CONFIG &conf);

    void setMagComp(QVector<double> comp);
    void setMagCompCenter(QVector<double> center);

signals:
    void loadMagCal();

private slots:
    void on_magCalLoadButton_clicked();

private:
    Ui::ConfCommonWidget *ui;
};

#endif // CONFCOMMONWIDGET_H
