#include "roboGLWidget.h"

#include <QPainter>
roboGLWidget::roboGLWidget(QWidget *parent) : QOpenGLWidget(parent)
{
}

roboGLWidget::~roboGLWidget()
{
}
void roboGLWidget::paintEvent()
{
    QPainter painter;
    painter.begin(this);
    painter.end();
}
