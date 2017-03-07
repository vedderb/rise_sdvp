#ifndef MAGCAL_H
#define MAGCAL_H

#include <QWidget>

namespace Ui {
class MagCal;
}

class MagCal : public QWidget
{
    Q_OBJECT

public:
    explicit MagCal(QWidget *parent = 0);
    ~MagCal();

    void addSample(double x, double y, double z);
    bool calculateCompensation();

    double getCx();
    double getCy();
    double getCz();

    double getXx();
    double getXy();
    double getXz();

    double getYx();
    double getYy();
    double getYz();

    double getZx();
    double getZy();
    double getZz();

private slots:
    void timerSlot();

    void on_magSampleClearButton_clicked();
    void on_magReplotButton_clicked();
    void on_magCalcCompButton_clicked();
    void on_magOpenFileButton_clicked();
    void on_magSampleSaveButton_clicked();

private:
    Ui::MagCal *ui;
    QTimer *mTimer;
    bool mMagReplot;

    QVector<QVector<double> > mMagSamples;
    QVector<double> mMagComp;
    QVector<double> mMagCompCenter;

    void loadMagPoints(QString path);
    void plotMagPoints();
    void calcMagComp();
    void updateMagPlots();
    void clearMagPlots();
};

#endif // MAGCAL_H
