#include "correctionanalysis.h"
#include "ui_correctionanalysis.h"
#include <QFileDialog>
#include <QStringList>

CorrectionAnalysis::CorrectionAnalysis(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CorrectionAnalysis)
{
    ui->setupUi(this);

    ui->logPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->logPlot->legend->setVisible(true);
    ui->logPlot->xAxis->setLabel("Seconds (s)");
    ui->logPlot->yAxis->setLabel("Value");
    ui->logPlot->yAxis2->setLabel("Corr Error (m)");
    ui->logPlot->yAxis2->setVisible(true);
}

CorrectionAnalysis::~CorrectionAnalysis()
{
    delete ui;
}

void CorrectionAnalysis::on_logFileChooseButton_clicked()
{
    QString path;
    path = QFileDialog::getOpenFileName(this, tr("Choose log file to open"));
    if (path.isNull()) {
        return;
    }

    ui->logFileEdit->setText(path);
}

void CorrectionAnalysis::on_logFileOpenButton_clicked()
{
    QFile file(ui->logFileEdit->text());
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::critical(this, "Open Log File",
                              "Could not open\n" + ui->logFileEdit->text() + "\nfor reading");
        return;
    }

    mCorrVector.clear();

    QTextStream stream(&file);

    while (!stream.atEnd()) {
        QString line = stream.readLine();
        if (line.toUpper().startsWith("CORR")) {
            QStringList tokens = line.split(" ");
            CORR_DATA_T corr;
            corr.time = tokens.at(1).toDouble() / 1000.0;
            corr.fix_type = tokens.at(2).toInt();
            corr.x = tokens.at(3).toDouble();
            corr.y = tokens.at(4).toDouble();
            corr.yaw = tokens.at(5).toDouble();
            corr.speed = tokens.at(6).toDouble();
            corr.corr_diff = tokens.at(7).toDouble();
            mCorrVector.append(corr);
        }
    }

    file.close();

    QVector<double> time;
    QVector<double> speed;
    QVector<double> yaw;
    QVector<double> corr_diff;

    for (CORR_DATA_T c: mCorrVector) {
        time.append(c.time);
        speed.append(c.speed);
        yaw.append(c.yaw);
        corr_diff.append(c.corr_diff);
    }

    ui->logPlot->clearGraphs();
    ui->logPlot->addGraph();
    ui->logPlot->xAxis->setRangeReversed(true);
    ui->logPlot->graph()->setPen(QPen(Qt::black));
    ui->logPlot->graph()->setData(time, speed);
    ui->logPlot->graph()->setName(tr("Speed"));
    ui->logPlot->addGraph();
    ui->logPlot->graph()->setPen(QPen(Qt::green));
    ui->logPlot->graph()->setData(time, yaw);
    ui->logPlot->graph()->setName(tr("Yaw"));
    ui->logPlot->addGraph(ui->logPlot->xAxis, ui->logPlot->yAxis2);
    ui->logPlot->graph()->setPen(QPen(Qt::blue));
    ui->logPlot->graph()->setData(time, corr_diff);
    ui->logPlot->graph()->setName(tr("Corr Diff"));
    ui->logPlot->rescaleAxes();
    ui->logPlot->replot();
}
