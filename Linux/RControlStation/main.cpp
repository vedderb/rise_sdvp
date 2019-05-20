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
    qDebug() << "--plotroutessize [w:h] : Size to plot with in pixels";
    qDebug() << "--plotroutesformat [format] : plot format; PDF or PNG";
    qDebug() << "--plotroutesmargins [margins] : Margins around the plot";
    qDebug() << "--plotroutesselect [id] : Select route with id";
    qDebug() << "--plotroutesshowgrid : Show grid in plot";
    qDebug() << "--plotroutesshowtext : Show text next to route points";
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
    a.setApplicationName("RC Car Tool");

    QStringList args = QCoreApplication::arguments();

    typedef enum {
        PLOT_ROUTE_FORMAT_PDF = 0,
        PLOT_ROUTE_FORMAT_PNG
    } PLOT_ROUTE_FORMAT;

    QString plotRoutesFile;
    int plotRoutesW = 640;
    int plotRoutesH = 480;
    PLOT_ROUTE_FORMAT plotRoutesFormat = PLOT_ROUTE_FORMAT_PDF;
    double plotRoutesMargins = 0.1;
    int plotRoutesSelect = -1;
    bool plotRoutesShowGrid = false;
    bool plotRoutesShowText = false;

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
            if ((i - 1) < args.size()) {
                i++;
                plotRoutesFile = args.at(i);
                found = true;
            }
        }

        if (str == "--plotroutessize") {
            if ((i - 1) < args.size()) {
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
            if ((i - 1) < args.size()) {
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
            if ((i - 1) < args.size()) {
                i++;
                plotRoutesMargins = args.at(i).toDouble();
                found = true;
            }
        }

        if (str == "--plotroutesselect") {
            if ((i - 1) < args.size()) {
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

        return a.exec();
    }
}
