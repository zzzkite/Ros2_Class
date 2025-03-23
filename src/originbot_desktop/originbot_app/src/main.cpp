#include <QApplication>
#include <QMovie>
#include <QPixmap>
#include <QSplashScreen>
#include <iostream>

#include "mainwindow.h"

#define START_TIME 2000

int main(int argc, char *argv[])
{

    QApplication App(argc, argv);

    // 启动开启动画
    QPixmap pixmap("://background/loding5.gif");
    QSplashScreen splash(pixmap);
    splash.setWindowOpacity(1);
    QLabel label(&splash);
    QMovie movie("://background/loding5.gif");
    label.setMovie(&movie);
    movie.start();
    splash.show();
    splash.setCursor(Qt::BlankCursor);
    for (int count = 0; count < START_TIME; count += movie.speed())
    {
        App.processEvents();
        QThread::msleep(50);
    }

    // 设置自定义类型
    qRegisterMetaType<RobotPose>("RobotPose");
    qRegisterMetaType<RobotSpeed>("RobotSpeed");
    qRegisterMetaType<RobotState>("RobotState");

    // 启动主窗口
    MainWindow window;
    window.show();
    splash.finish(&window); // 在主体对象初始化完成后结束启动动画
    return App.exec();
}
