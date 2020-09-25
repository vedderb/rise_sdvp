#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QSerialPort>
#include <QLabel>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void serialDataAvailable();
    void serialPortError(QSerialPort::SerialPortError error);
    void timerSlot();
    void showStatusInfo(QString info, bool isGood);

    void on_serialRefreshButton_clicked();
    void on_serialConnectButton_clicked();
    void on_setUpdateRateButton_clicked();
    void on_serialDisconnectButton_clicked();

private:
    Ui::MainWindow *ui;
    QTimer *mTimer;
    QSerialPort *mSerialPort;
    QLabel *mStatusLabel;
    int mStatusInfoTime;

    void sendPacket(QByteArray payload);

};

#endif // MAINWINDOW_H
