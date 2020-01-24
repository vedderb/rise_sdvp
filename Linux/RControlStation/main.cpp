/*
    Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mainwindow.h"
#include <QApplication>
#include <QStyleFactory>
#include "utility.h"
#include "mapwidget.h"

void showHelp()
{
    qDebug() << "Arguments";
    qDebug() << "-h, --help : Show help text";
    qDebug() << "--plotroutes [xmlFile] : Plot routes from XML file";
    qDebug() << "--plotroutessize [w]:[h] : Size to plot with in pixels";
    qDebug() << "--plotroutesformat [format] : plot format; PDF or PNG";
    qDebug() << "--plotroutesmargins [margins] : Margins around the plot";
    qDebug() << "--plotroutesselect [id] : Select route with id";
    qDebug() << "--plotroutesshowgrid : Show grid in plot";
    qDebug() << "--plotroutesshowtext : Show text next to route points";
    qDebug() << "--addcar [id]:[pollData] : Add car tab with car id";
    qDebug() << "--connectjs [dev] : Connect to joystick dev (e.g. /dev/input/js0)";
    qDebug() << "--connecttcp [ip]:[port] : Connect to car over TCP (multiple connections possible)";
    qDebug() << "--xmltcpserver [port] : Run XML TCP server on port";
    qDebug() << "--xmludpserver [port] : Run XML UDP server on port";
    qDebug() << "--loadroutes [xmlFile] : Load routes from XML file";
}

int main(int argc, char *argv[])
{
    //qputenv("QT_SCALE_FACTOR", QByteArray("1.5"));

    QApplication a(argc, argv);
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    // Style
    a.setStyleSheet("");
    a.setStyle(QStyleFactory::create("Fusion"));

    // Settings
    a.setOrganizationName("RISE");
    a.setOrganizationDomain("ri.se");
    a.setApplicationName("RControlStation");

    QStringList args = QCoreApplication::arguments();

    typedef enum {
        PLOT_ROUTE_FORMAT_PDF = 0,
        PLOT_ROUTE_FORMAT_PNG
    } PLOT_ROUTE_FORMAT;

    struct CarConn {
        QString ip;
        int port;
    };

    struct CarToAdd {
        int id;
        bool pollData;
    };

    QString plotRoutesFile;
    int plotRoutesW = 640;
    int plotRoutesH = 480;
    PLOT_ROUTE_FORMAT plotRoutesFormat = PLOT_ROUTE_FORMAT_PDF;
    double plotRoutesMargins = 0.1;
    int plotRoutesSelect = -1;
    bool plotRoutesShowGrid = false;
    bool plotRoutesShowText = false;
    QVector<CarToAdd> carsToAdd;
    QString jsStr;
    QList<CarConn> carsToConn;
    int xmlTcpPort = -2;
    int xmlUdpPort = -2;
    QString loadRoutesFile;

    for (int i = 0;i < args.size();i++) {
        // Skip the program argument
        if (i == 0) {
            continue;
        }

        QString str = args.at(i).toLower();

        // Skip path argument
        if (i >= args.size() && args.size() >= 3) {
            break;
        }

        bool dash = str.startsWith("-") && !str.startsWith("--");
        bool found = false;

        if ((dash && str.contains('h')) || str == "--help") {
            showHelp();
            found = true;
            return 0;
        }

        if (str == "--plotroutes") {
            if ((i + 1) < args.size()) {
                i++;
                plotRoutesFile = args.at(i);
                found = true;
            }
        }

        if (str == "--plotroutessize") {
            if ((i + 1) < args.size()) {
                i++;
                QString tmp = args.at(i);
                QStringList numbers = tmp.split(":");

                if (numbers.size() == 2) {
                    found = true;
                    plotRoutesW = numbers.at(0).toInt();
                    plotRoutesH = numbers.at(1).toInt();
                }
            }
        }

        if (str == "--plotroutesformat") {
            if ((i + 1) < args.size()) {
                i++;

                QString str2 = args.at(i).toLower();

                if (str2 == "pdf") {
                    plotRoutesFormat = PLOT_ROUTE_FORMAT_PDF;
                } else if (str2 == "png") {
                    plotRoutesFormat = PLOT_ROUTE_FORMAT_PNG;
                } else {
                    qCritical() << "Invalid format:" << str2;
                    return 1;
                }

                found = true;
            }
        }

        if (str == "--plotroutesmargins") {
            if ((i + 1) < args.size()) {
                i++;
                plotRoutesMargins = args.at(i).toDouble();
                found = true;
            }
        }

        if (str == "--plotroutesselect") {
            if ((i + 1) < args.size()) {
                i++;
                plotRoutesSelect = args.at(i).toInt();
                found = true;
            }
        }

        if (str == "--plotroutesshowgrid") {
            plotRoutesShowGrid = true;
            found = true;
        }

        if (str == "--plotroutesshowtext") {
            plotRoutesShowText = true;
            found = true;
        }

        if (str == "--addcar") {
            if ((i + 1) < args.size()) {
                i++;
                QString tmp = args.at(i);
                QStringList numbers = tmp.split(":");

                if (numbers.size() == 1) {
                    found = true;
                    CarToAdd c;
                    c.id = numbers.at(0).toInt();
                    c.pollData = false;
                    carsToAdd.append(c);
                } else if (numbers.size() == 2) {
                    found = true;
                    CarToAdd c;
                    c.id = numbers.at(0).toInt();
                    c.pollData = numbers.at(1).toInt();
                    carsToAdd.append(c);
                }
            }
        }

        if (str == "--connectjs") {
            if ((i + 1) < args.size()) {
                i++;
                jsStr = args.at(i);
                found = true;
            }
        }

        if (str == "--connecttcp") {
            if ((i + 1) < args.size()) {
                i++;
                QString tmp = args.at(i);
                QStringList numbers = tmp.split(":");

                if (numbers.size() == 1) {
                    found = true;
                    CarConn c;
                    c.ip = numbers.at(0);
                    c.port = 8300;
                    carsToConn.append(c);
                } else if (numbers.size() == 2) {
                    found = true;
                    CarConn c;
                    c.ip = numbers.at(0);
                    c.port = numbers.at(1).toInt();
                    carsToConn.append(c);
                }
            }
        }

        if (str == "--xmltcpserver") {
            if ((i + 1) < args.size()) {
                i++;
                xmlTcpPort = args.at(i).toInt();
                found = true;
            }
        }

        if (str == "--xmludpserver") {
            if ((i + 1) < args.size()) {
                i++;
                xmlUdpPort = args.at(i).toInt();
                found = true;
            }
        }

        if (str == "--loadroutes") {
            if ((i + 1) < args.size()) {
                i++;
                loadRoutesFile = args.at(i);
                found = true;
            }
        }

        if (!found) {
            if (dash) {
                qCritical() << "At least one of the flags is invalid:" << str;
            } else {
                qCritical() << "Invalid option:" << str;
            }

            showHelp();
            return 1;
        }
    }

    if (!plotRoutesFile.isEmpty()) {
        MapWidget map;
        int res = utility::loadRoutes(plotRoutesFile, &map);

        if (res >= 0) {
            map.zoomInOnRoute(-1, plotRoutesMargins, plotRoutesW, plotRoutesH);
            map.setDrawGrid(plotRoutesShowGrid);
            map.setDrawRouteText(plotRoutesShowText);

            if (plotRoutesSelect >= 0) {
                map.setRouteNow(plotRoutesSelect);
            } else {
                map.setRouteNow(map.getRouteNum() - 1);
            }

            if (plotRoutesFormat == PLOT_ROUTE_FORMAT_PDF) {
                map.printPdf(plotRoutesFile + ".pdf", plotRoutesW, plotRoutesH);
            } else {
                map.printPng(plotRoutesFile + ".png", plotRoutesW, plotRoutesH);
            }
        } else {
            qCritical() << "Could not load routes from" << plotRoutesFile;
        }
    } else {
        MainWindow w;
        w.show();

        if (!loadRoutesFile.isEmpty()) {
            MapWidget *map = w.map();
            int res = utility::loadRoutes(loadRoutesFile, map);

            if (res >= 0) {
                map->zoomInOnRoute(-1, 0.8, 400, 400);
            } else {
                qCritical() << "Could not load routes from" << plotRoutesFile;
            }
        }

        for (auto c: carsToAdd) {
            w.addCar(c.id, c.pollData);
        }

        if (!jsStr.isEmpty()) {
            w.connectJoystick(jsStr);
        }

        for (auto c: carsToConn) {
            w.addTcpConnection(c.ip, c.port);
        }

        if (xmlTcpPort >= -1) {
            w.setNetworkTcpEnabled(true, xmlTcpPort);
        }

        if (xmlUdpPort >= -1) {
            w.setNetworkUdpEnabled(true, xmlUdpPort);
        }

        return a.exec();
    }
}
