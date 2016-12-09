#include "mainwindow.h"
#include <QApplication>
#include <QStyleFactory>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Style
    a.setStyleSheet("");
    a.setStyle(QStyleFactory::create("Fusion"));

    // Settings
    a.setOrganizationName("SP");
    a.setOrganizationDomain("sp.se");
    a.setApplicationName("Car Network Tester");

    MainWindow w;
    w.show();

    return a.exec();
}
