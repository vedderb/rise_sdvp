#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QUdpSocket>

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
    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);
    void udpReadReady();

    void on_getStateGenerateButton_clicked();
    void on_addRoutePointGenerateButton_clicked();
    void on_sendButton_clicked();
    void on_clearButton_clicked();
    void on_disconnectedButton_toggled(bool checked);
    void on_tcpButton_toggled(bool checked);
    void on_udpButton_toggled(bool checked);

private:
    Ui::MainWindow *ui;
    QUdpSocket *mUdpSocket;
    QTcpSocket *mTcpSocket;
    bool mTcpConnected;

};

#endif // MAINWINDOW_H
