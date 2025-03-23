#ifndef ROBOT_GLWIDGET
#define ROBOT_GLWIDGET
#include <QOpenGLWidget>
class roboGLWidget : public QOpenGLWidget
{
    Q_OBJECT
  private:
    void paintEvent();

  public:
    roboGLWidget(QWidget *parent);
    ~roboGLWidget();
};

#endif // !ROBOT_GLWIDGET
