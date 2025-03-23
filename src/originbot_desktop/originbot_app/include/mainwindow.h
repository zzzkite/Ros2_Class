#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QDateTime>
#include <QMainWindow>

#include "QTimer"

#include "rclcomm.h"
#include "roboGLWidget.h"
#include "roboImg.h"
#include "roboItem.h"
#include "ui_mainwindow.h"
QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    rclcomm *commNode;
  public slots:
    void updateRobotPose(RobotPose pose);
    void updateOdomInfo(RobotState);

  private:
    void initUi();
    void closeEvent(QCloseEvent *event); // Overloaded function
    void setCurrentMenu(QPushButton *cur_btn);

  private:
    Ui::MainWindowDesign *ui;
    QGraphicsScene *m_qGraphicScene = nullptr;
    roboItem *m_roboItem = nullptr;
    roboImg *m_roboImg = nullptr;
    QTimer *m_timerCurrentTime;
    roboGLWidget *m_roboGLWidget;
};
#endif // MAINWINDOW_H
