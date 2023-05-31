#include "QRobotItem.h"

QRobotItem::QRobotItem(QObject *parent)
        : QObject{parent} {
    setAcceptHoverEvents(true);
    setAcceptedMouseButtons(Qt::AllButtons);
    setAcceptDrops(true);
    setFlag(ItemAcceptsInputMethod, true);
    moveBy(0, 0);
    moveCursor = new QCursor(QPixmap(":/image/data/images/cursor_move.png"), 0, 0);
    set2DPoseCursor = new QCursor(QPixmap(":/image/data/images/cursor_pos.png"), 0, 0);
    set2DGoalCursor = new QCursor(QPixmap(":/image/data/images/cursor_pos.png"), 0, 0);
    setRobotColor(AppEnums::QRobotColor::Blue);
    setDefault();
}

QRobotItem::~QRobotItem() {

}

void QRobotItem::setRobotColor(AppEnums::QRobotColor color) {
    switch (color) {
        case AppEnums::QRobotColor::Blue: {
            robotImg.load(":/image/data/images/Navigate.png");
        }
            break;
        case AppEnums::QRobotColor::Red: {
            robotImg.load(":/image/data/images/robot_red.png");
        }
            break;
        case AppEnums::QRobotColor::Yellow: {
            robotImg.load(":/image/data/images/robot_yellow.png");
        }
            break;
    }
    QMatrix matrix;
    matrix.rotate(90);
    robotImg = robotImg.transformed(matrix, Qt::SmoothTransformation);
}

void QRobotItem::setRobotSize(QSize size) {
    robotImg = robotImg.scaled(size);
}

int QRobotItem::QColorToInt(const QColor &color) {
    return (int) (((unsigned int) color.blue() << 16) |
                  (unsigned short) (((unsigned short) color.green() << 8) |
                                    color.red()));
}

void QRobotItem::paintImage(int id, QImage image) {
    m_image = image;
}

void QRobotItem::paintLaserScan(QPolygonF points) {
    laserPoints = points;
    update();
}

void QRobotItem::paintPlannerPath(QPolygonF path) {
    plannerPath = path;
    update();
}

void QRobotItem::paintMaps(QImage map) {
    m_imageMap = map;
    update();
}

void QRobotItem::paintRoboPos(QRobotPose pos) {
    //  CONSOLE << "pos:" << pos.x << " " << pos.y << " " << pos.theta;
    RoboPostion = QPointF(pos.x, pos.y);
    m_roboYaw = pos.theta;
    update();
}

void QRobotItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                       QWidget *widget) {
    drawMap(painter);
    drawRoboPos(painter);
    drawPlannerPath(painter);
    drawLaserScan(painter);
    drawTools(painter);
}

void QRobotItem::drawTools(QPainter *painter) {
    if (currCursor == set2DPoseCursor || currCursor == set2DGoalCursor) {

        if (m_pressedPoint.x() != 0 && m_pressedPoint.y() != 0 &&
            m_pressingPoint.x() != 0 && m_pressingPoint.y() != 0) {
            painter->setPen(QPen(QColor(0, 255, 0, 255), 2));

            double theta = qAtan((m_pressingPoint.y() - m_pressedPoint.y()) /
                                 (m_pressingPoint.x() - m_pressedPoint.x()));

            double dy = sin(theta) * 20;
            double dx = cos(theta) * 20;
            QPointF startPoint, endPoint;
            startPoint = m_pressedPoint;
            if (m_pressingPoint.x() - m_pressedPoint.x() > 0) {
                endPoint = QPointF(m_pressedPoint.x() + dx, m_pressedPoint.y() + dy);
            } else {
                endPoint = QPointF(m_pressedPoint.x() - dx, m_pressedPoint.y() - dy);
            }
            QLineF line(startPoint, endPoint);
            painter->drawLine(line);
            QLineF v = line.unitVector();
            if (!v.isNull()) {
                v.setLength(5);
                v.translate(QPointF(line.dx(), line.dy()));
                QLineF n = v.normalVector();
                n.setLength(n.length() * 0.5);
                QLineF n2 = n.normalVector().normalVector();
                QPointF p1 = v.p2();
                QPointF p2 = n.p2();
                QPointF p3 = n2.p2();
                // painter->setBrush(QBrush(color));
                if (p1.isNull() == false && p2.isNull() == false) {
                    QLineF lineA(p1, p2);
                    if (lineA.length() > 4) {
                        painter->drawLine(lineA);
                    }
                }
                if (p2.isNull() == false && p3.isNull() == false) {
                    QLineF lineB(p2, p3);
                    if (lineB.length() > 4) {
                        painter->drawLine(lineB);
                    }
                }
                if (p3.isNull() == false && p1.isNull() == false) {
                    QLineF lineC(p3, p1);
                    if (lineC.length() > 4) {
                        painter->drawLine(lineC);
                    }
                }
            }
        }
    }
}

void QRobotItem::drawMap(QPainter *painter) {
    painter->drawImage(0, 0, m_imageMap);
}

void QRobotItem::drawRoboPos(QPainter *painter) {
    painter->setPen(QPen(QColor(255, 0, 0, 255), 1, Qt::SolidLine, Qt::RoundCap,
                         Qt::RoundJoin));
    painter->save();
    painter->translate(RoboPostion.x(), RoboPostion.y());
    painter->rotate(rad2deg(-m_roboYaw));
    painter->drawPoint(QPoint(0, 0));
    painter->drawPixmap(QPoint(-robotImg.width() / 2, -robotImg.height() / 2),
                        robotImg);
    painter->restore();
}

void QRobotItem::drawLaserScan(QPainter *painter) {
    painter->setPen(QPen(QColor(255, 0, 0, 255), 1));
    painter->drawPoints(laserPoints);
}

void QRobotItem::drawPlannerPath(QPainter *painter) {
    painter->setPen(QPen(QColor(0, 0, 0, 255), 1));
    painter->drawPoints(plannerPath);
}

void QRobotItem::setMax() {
    m_scaleValue *= 1.1;
    setScale(m_scaleValue);
}

void QRobotItem::setMin() {
    m_scaleValue *= 0.9;
    setScale(m_scaleValue);
}

void QRobotItem::setDefault() {
    this->setScale(defaultScale);
    this->moveBy(0, 0);
    m_scaleValue = defaultScale;
}

QRectF QRobotItem::boundingRect() const {
    return QRectF(0, 0, m_imageMap.width(), m_imageMap.height());
}

void QRobotItem::move(double x, double y) {
    this->moveBy(x, y);
}

// mouse event
void QRobotItem::wheelEvent(QGraphicsSceneWheelEvent *event) {
    this->setCursor(Qt::CrossCursor);
    if ((event->delta() > 0) && (m_scaleValue >= 50)) {
        return;
    } else if ((event->delta() < 0) &&
               (m_scaleValue <=
                m_scaleDafault)) {
        // ResetItemPos();
    } else {
        qreal qrealOriginScale = m_scaleValue;
        if (event->delta() > 0) {
            m_scaleValue *= 1.1;
        } else {
            m_scaleValue *= 0.9;
        }
        setScale(m_scaleValue);
        if (event->delta() > 0) {
            moveBy(-event->pos().x() * qrealOriginScale * 0.1,
                   -event->pos().y() * qrealOriginScale *
                   0.1);
        } else {
            moveBy(event->pos().x() * qrealOriginScale * 0.1,
                   event->pos().y() * qrealOriginScale *
                   0.1);
        }
    }
}

void QRobotItem::slot_set2DPos() {
    this->setCursor(*set2DPoseCursor);
    currCursor = set2DPoseCursor;
}

void QRobotItem::slot_set2DGoal() {
    this->setCursor(*set2DGoalCursor);
    currCursor = set2DGoalCursor;
}

void QRobotItem::slot_setMoveCamera() {
    this->setCursor(*moveCursor);
    currCursor = moveCursor;
}

void QRobotItem::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        if (currCursor != moveCursor) {
            m_pressedPoint = event->pos();
        }
        m_startPos = event->pos();
        m_isPress = true;
    } else if (event->button() == Qt::RightButton) {
        // ResetItemPos();
    }
    update();
}

void QRobotItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
    m_pressingPoint = event->pos();

    if (currCursor == NULL) {
        this->setCursor(*moveCursor);
        currCursor = moveCursor;
    }

    if (m_isPress && currCursor == moveCursor) {
        QPointF point = (event->pos() - m_startPos) * m_scaleValue;
        moveBy(point.x(), point.y());
    }
    update();
}

void QRobotItem::hoverMoveEvent(QGraphicsSceneHoverEvent *event) {
    emit cursorPos(event->pos());
}

void QRobotItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    m_isPress = false;

    if (currCursor == set2DPoseCursor) {
        m_isOtherCursor = false;
        QRobotPose target_pos;
        target_pos.x = m_pressedPoint.x();
        target_pos.y = m_pressedPoint.y();
        target_pos.theta = ::getAngle(m_pressedPoint.x(), m_pressedPoint.y(),
                                      m_pressingPoint.x(), m_pressingPoint.y());
        emit signalPub2DPos(target_pos);
        m_pressedPoint = QPointF(0, 0);
        m_pressingPoint = QPointF(0, 0);
        this->setCursor(*moveCursor);
        currCursor = moveCursor;
    } else if (currCursor == set2DGoalCursor) {
        m_isOtherCursor = false;
        QRobotPose init_pos;
        init_pos.x = m_pressedPoint.x();
        init_pos.y = m_pressedPoint.y();
        init_pos.theta = ::getAngle(m_pressedPoint.x(), m_pressedPoint.y(),
                                    m_pressingPoint.x(), m_pressingPoint.y());
        emit signalPub2DGoal(init_pos);
        m_pressedPoint = QPointF(0, 0);
        m_pressingPoint = QPointF(0, 0);
        this->setCursor(*moveCursor);
        currCursor = moveCursor;
    }
}



