#include "HelloMotionModule.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    HelloMotionModule w;
    w.show();

    return a.exec();
}
