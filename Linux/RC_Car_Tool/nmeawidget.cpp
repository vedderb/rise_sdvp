#include "nmeawidget.h"
#include "ui_nmeawidget.h"
#include "nmeaserver.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QTextStream>

NmeaWidget::NmeaWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::NmeaWidget)
{
    ui->setupUi(this);
    layout()->setContentsMargins(0, 0, 0, 0);

    mNmeaForwardServer = new TcpBroadcast(this);
}

NmeaWidget::~NmeaWidget()
{
    delete ui;
}

void NmeaWidget::inputNmea(QByteArray msg)
{
    if (ui->nmeaPrintBox->isChecked()) {
        ui->nmeaBrowser->append(QString::fromLocal8Bit(msg));
    }

    mNmeaForwardServer->broadcastData(msg);

    NmeaServer::nmea_gga_info_t gga;
    QTextStream msgs(msg);

    while(!msgs.atEnd()) {
        QString str = msgs.readLine();
        QByteArray data = str.toLocal8Bit();

        // Hack
        if (str == "$GPGSA,A,1,,,,,,,,,,,,,,,*1E") {
            ui->nmeaFixTypeLabel->setText("Solution: Invalid");
            ui->nmeaSatsLabel->setText("Satellites: 0");
        }

        if (NmeaServer::decodeNmeaGGA(data, gga) >= 0) {
            QString satStr;
            satStr.sprintf("Satellites: %d", gga.n_sat);
            ui->nmeaSatsLabel->setText(satStr);

            QString fix_type;
            switch (gga.fix_type) {
            case 0: fix_type = "Solution: Invalid"; break;
            case 1: fix_type = "Solution: SPP"; break;
            case 2: fix_type = "Solution: DGPS"; break;
            case 3: fix_type = "Solution: PPS"; break;
            case 4: fix_type = "Solution: RTK Fix"; break;
            case 5: fix_type = "Solution: RTK Float"; break;
            default: fix_type = "Solution: Unknown"; break;
            }

            ui->nmeaFixTypeLabel->setText(fix_type);
            ui->nmeaCorrAgeLabel->setText(QString("Corr age: %1 s").arg(gga.diff_age));
        }
    }
}

void NmeaWidget::on_nmeaLogChooseButton_clicked()
{
    QString path;
    path = QFileDialog::getSaveFileName(this, tr("Choose where to save the NMEA log"));
    if (path.isNull()) {
        return;
    }

    ui->nmeaLogEdit->setText(path);
}

void NmeaWidget::on_nmeaLogActiveBox_toggled(bool checked)
{
    if (checked) {
        bool ok = mNmeaForwardServer->logToFile(ui->nmeaLogEdit->text());

        if (!ok) {
            QMessageBox::warning(this, "NMEA Log",
                                 "Could not open log file.");
            ui->nmeaLogActiveBox->setChecked(false);
        }
    } else {
        mNmeaForwardServer->logStop();
    }
}

void NmeaWidget::on_nmeaServerActiveBox_toggled(bool checked)
{
    if (checked) {
        if (!mNmeaForwardServer->startTcpServer(ui->nmeaServerPortBox->value())) {
            QMessageBox::warning(this, "TCP Server Error",
                                 "Creating TCP server for NMEA data failed. Make sure that the port is not "
                                 "already in use.");
            ui->nmeaServerActiveBox->setChecked(false);
        }
    } else {
        mNmeaForwardServer->stopServer();
    }
}
