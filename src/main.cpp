#include <QtGui>
#include <QApplication>
#include "nav_ui/mainwindow.hpp"

int main(int argc, char **argv) {

    //Starting Qt
    QApplication app(argc, argv);
    nav_ui::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
