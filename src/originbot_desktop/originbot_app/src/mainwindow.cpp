#include "mainwindow.h"

#include "RobotAlgorithm.h"

#define Odom_rate 0.2
#define panel_rate 0.002

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
    ui->setupUi(this);
    // 设置通信相关节点
    commNode = new rclcomm();

    // 初始化场景类
    m_qGraphicScene = new QGraphicsScene();
    m_qGraphicScene->clear();

    // 初始化item
    m_roboItem = new roboItem();
    m_roboImg = new roboImg();
    m_roboItem->setZValue(1);
    m_roboImg->setZValue(10);

    // 视图中添加Item
    m_qGraphicScene->addItem(m_roboItem);
    m_qGraphicScene->addItem(m_roboImg);
    // ui中的graphicsView添加场景
    ui->mapViz->setScene(m_qGraphicScene);
    ui->mapViz->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    // 设置相应控件通信
    connect(commNode, SIGNAL(emitUpdateMap(QImage)), m_roboItem, SLOT(updateMap(QImage)));
    connect(commNode, SIGNAL(emitUpdateLocalCostMap(QImage, RobotPose)), m_roboItem,
            SLOT(updateLocalCostMap(QImage, RobotPose)));
    connect(commNode, SIGNAL(emitUpdateGlobalCostMap(QImage)), m_roboItem, SLOT(updateGlobalCostMap(QImage)));
    connect(commNode, SIGNAL(emitUpdateRobotPose(RobotPose)), this, SLOT(updateRobotPose(RobotPose)));
    connect(commNode, SIGNAL(emitUpdateLaserPoint(QPolygonF)), m_roboItem, SLOT(updateLaserPoints(QPolygonF)));
    connect(commNode, SIGNAL(emitUpdatePath(QPolygonF)), m_roboItem, SLOT(updatePath(QPolygonF)));
    connect(commNode, SIGNAL(emitUpdateLocalPath(QPolygonF)), m_roboItem, SLOT(updateLocalPath(QPolygonF)));
    connect(commNode, SIGNAL(emitOdomInfo(RobotState)), this, SLOT(updateOdomInfo(RobotState)));
    connect(m_roboItem, SIGNAL(signalPub2DPose(QPointF, QPointF)), commNode, SLOT(pub2DPose(QPointF, QPointF)));
    connect(m_roboItem, SIGNAL(signalPub2DGoal(QPointF, QPointF)), commNode, SLOT(pub2DGoal(QPointF, QPointF)));

    // ui相关
    connect(ui->pushButton_i, SIGNAL(clicked()), commNode, SLOT(slot_cmd_control()));
    connect(ui->pushButton_back, SIGNAL(clicked()), commNode, SLOT(slot_cmd_control()));
    connect(ui->pushButton_l, SIGNAL(clicked()), commNode, SLOT(slot_cmd_control()));
    connect(ui->pushButton_j, SIGNAL(clicked()), commNode, SLOT(slot_cmd_control()));
    //     connect(ui->pushButton_u,SIGNAL(clicked()),commNode,
    //             SLOT(slot_cmd_control()));
    //     connect(ui->pushButton_o,SIGNAL(clicked()),commNode,
    //             SLOT(slot_cmd_control()));
    //     connect(ui->pushButton_m,SIGNAL(clicked()),commNode,
    //             SLOT(slot_cmd_control()));
    //     connect(ui->pushButton_mm,SIGNAL(clicked()),commNode,
    //             SLOT(slot_cmd_control()));
    connect(ui->pushButton_s, SIGNAL(clicked()), commNode, SLOT(slot_cmd_control()));

    connect(ui->horizontalSlider_linear, SIGNAL(valueChanged(int)), commNode, SLOT(GETHORIZONTALSLIADER_LINEAR(int)));
    connect(ui->horizontalSlider_raw, SIGNAL(valueChanged(int)), commNode, SLOT(GETHORIZONTALSLIADER_RAW(int)));
    connect(ui->set_pos_btn, &QPushButton::clicked, [=]() { m_roboItem->start2DPose(); });
    connect(ui->set_goal_btn, &QPushButton::clicked, [=]() { m_roboItem->start2DGoal(); });
    connect(ui->close_btn, &QPushButton::clicked, [=]() { this->close(); });
    connect(ui->min_btn, &QPushButton::clicked, [=]() { this->showMinimized(); });

    connect(ui->max_btn, &QPushButton::clicked, [=]() {
        if (this->isFullScreen())
        {
            this->showNormal();
        }
        else
        {
            this->showFullScreen();
        }
    });

    connect(m_roboItem, &roboItem::cursorPos, [=](QPointF pos) {
        QPointF mapPos = commNode->transScenePoint2Word(pos);
        ui->label_pos_map->setText("x: " + QString::number(mapPos.x()).mid(0, 4) +
                                   "  y: " + QString::number(mapPos.y()).mid(0, 4));
        ui->label_pos_scene->setText("x: " + QString::number(pos.x()).mid(0, 4) +
                                     "  y: " + QString::number(pos.y()).mid(0, 4));
    });

    m_timerCurrentTime = new QTimer;
    m_timerCurrentTime->setInterval(100);
    m_timerCurrentTime->start();
    QObject::connect(m_timerCurrentTime, &QTimer::timeout,
                     [=]() { ui->label_time->setText(QDateTime::currentDateTime().toString("  hh:mm:ss  ")); });
    commNode->start();
    initUi();
}

void MainWindow::updateRobotPose(RobotPose pose)
{
    m_roboItem->updateRobotPose(pose);
    QPointF pos;
    pos.setX(pose.x);
    pos.setY(pose.y);
    QPointF scenePose = m_roboItem->mapToScene(pos);
    m_roboImg->updatePose(pose);
    m_roboImg->setPos(scenePose.x(), scenePose.y());
}

void MainWindow::updateOdomInfo(RobotState state)
{
    QString number = QString::number(abs(state.vx * 100)).mid(0, 2);
    if (number[1] == ".")
    {
        number = number.mid(0, 1);
    }
    ui->label_speed->setText("     OriginBot_Speed " + number + "CM/S");
}

void MainWindow::initUi()
{
    setWindowFlags(Qt::CustomizeWindowHint);
    ui->pushButton_status->setIcon(QIcon("://images/status/status_none.png"));
    ui->min_btn->setIcon(QIcon("://images/min.png"));
    ui->max_btn->setIcon(QIcon("://images/max.png"));
    ui->close_btn->setIcon(QIcon("://images/close.png"));
}
void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
    delete ui;
}
