#include <QApplication>

#include "qt_sim_window.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    qt_sim_window window;
    window.show();
    return app.exec();
}
