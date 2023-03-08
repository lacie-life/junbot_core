#ifndef QROBOTITEM_H
#define QROBOTITEM_H

#include <QColor>
#include <QCursor>
#include <QDebug>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include <QObject>
#include <QPainter>
#include <QPolygon>
#include <QTimer>
#include <QtMath>
#include <opencv2/highgui/highgui.hpp>

#include "QRobotUltis.h"
#include "AppConstants.h"

class QRobotItem : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    QRobotItem(QObject *parent = nullptr);
    ~QRobotItem();
    QRectF boundingRect() const;
    void wheelEvent(QGraphicsSceneWheelEvent *event);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void hoverMoveEvent(QGraphicsSceneHoverEvent *event);
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget);
    int QColorToInt(const QColor &color);
    void setRobotColor(AppEnums::QRobotColor color);
    void setRobotSize(QSize size);

    void get_version() { qDebug() << "1.0.0"; }
    void setMax();
    void setMin();
    void setDefault();
    void move(double x, double y);

public:
    QPolygon MapPoints;
    QPolygonF plannerPath;
    QPolygonF laserPoints;
    QPointF RoboPostion;
    QSizeF mapSize;
    QImage m_image;
    QImage m_imageMap;
    QTimer timer_update;
    int m_sizeCar = 4;
    double m_roboYaw;
    double m_roboR = 5;
    double map_size = 1;
    double defaultScale = 2;
    double PI = 3.1415926;

    QCursor *moveCursor = NULL;
    QCursor *set2DPoseCursor = NULL;
    QCursor *set2DGoalCursor = NULL;
    QCursor *currCursor = NULL;

signals:
    void cursorPos(QPointF);
    void signalPub2DPos(QRobotPose pose);
    void signalPub2DGoal(QRobotPose pose);

public slots:
    void paintMaps(QImage map);
    void paintRoboPos(QRobotPose pos);
    void paintImage(int, QImage);
    void paintPlannerPath(QPolygonF);
    void paintLaserScan(QPolygonF);
    void slot_set2DPos();
    void slot_set2DGoal();
    void slot_setMoveCamera();

private:
    void drawMap(QPainter *painter);
    void drawRoboPos(QPainter *painter);
    void drawLaserScan(QPainter *painter);
    void drawPlannerPath(QPainter *painter);
    void drawTools(QPainter *painter);

private:
    int m_zoomState;
    bool m_isPress;
    bool m_isOtherCursor{false};
    QPixmap robotImg;
    QPointF m_startPos;
    QPointF m_pressedPoint = QPointF(0, 0);
    QPointF m_pressingPoint = QPointF(0, 0);
    qreal m_scaleValue = 1;
    qreal m_scaleDafault = 1;

};

#endif // QROBOTITEM_H
