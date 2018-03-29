#ifndef CORRECTIONANALYSIS_H
#define CORRECTIONANALYSIS_H

#include <QWidget>
#include <QVector>

namespace Ui {
class CorrectionAnalysis;
}

class CorrectionAnalysis : public QWidget
{
    Q_OBJECT

public:
    explicit CorrectionAnalysis(QWidget *parent = 0);
    ~CorrectionAnalysis();

private slots:
    void on_logFileChooseButton_clicked();
    void on_logFileOpenButton_clicked();

private:
    Ui::CorrectionAnalysis *ui;

    struct CORR_DATA_T {
        double time;
        int fix_type;
        double x;
        double y;
        double yaw;
        double speed;
        double corr_diff;
    };

    QVector<CORR_DATA_T> mCorrVector;

};

#endif // CORRECTIONANALYSIS_H
