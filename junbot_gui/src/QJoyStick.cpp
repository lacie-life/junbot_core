#include "QJoyStick.h"
#include "AppConstants.h"

QJoyStick::QJoyStick(QWidget *parent)
  : QWidget{parent}
{
  setPalette(QPalette(Qt::white));
  resize(parent->width(), parent->height());
  setMinimumSize(100, 100);
  mouseX = width() / 2;
  mouseY = height() / 2;
  tim = new QTimer(this);
  connect(tim, &QTimer::timeout, this,
          [=] { emit keyNumChanged(getKeyNum()); });
  //    connect(this,&QJoyStick::keyNumchanged,this,[=](int num){
  //        qDebug()<<num<<endl;
  //    });
}

QJoyStick::~QJoyStick()
{

}

void QJoyStick::paintEvent(QPaintEvent *event)
{
  QPainter painter(this);

  int side = qMin(width(), height());

  padR = side / 2;       // Chassis radius
  padX = padR;           // Chassis center
  padY = padR;           // Chassis center
  JoyStickR = padR / 4;  // Rocker circle radius
  int JoyStickMaxR = padR - JoyStickR;
  QColor JoyStickColor;
  JoyStickColor.setRgb(85, 87, 83);

  // Load the chassis image
  //  painter.save();

  //  painter.scale(side / 400.0, side / 400.0); // The coordinates will scale with the window
  //  painter.drawPixmap(0, 0, QPixmap(":/image/pad.png"));
  //  painter.restore();

  // Self-drawn chassis
  painter.save();
  QRadialGradient RadialGradient(padR, padR, padR * 3, padR,
                                 padR);  // center 2, radius 1, focus 2
  RadialGradient.setColorAt(0, QColor(255, 253, 253, 255));  // Gradient
  RadialGradient.setColorAt(1, QColor(255, 240, 245, 190));  // Gradient
  painter.setBrush(RadialGradient);
  painter.setPen(Qt::NoPen);
  painter.drawEllipse(QPoint(padR, padR), side / 2, side / 2);  // big disc
  painter.restore();

  //  painter.drawText(20,20,tr("%1,%2,%3").arg(mouseX).arg(mouseY).arg(handPadDis));

  // When the mouse is not pressed, the joystick returns to the center of the chassis
  if (!mousePressed)
  {
    mouseX = padX;
    mouseY = padY;
  }

  handPadDis = Pointdis(padR, padR, mouseX, mouseY);

  if (handPadDis <= JoyStickMaxR)
  {
    JoyStickX = mouseX;
    JoyStickY = mouseY;
  }
  else
  {
    JoyStickX = (int)(JoyStickMaxR * (mouseX - padX) / handPadDis + padX);
    JoyStickY = (int)(JoyStickMaxR * (mouseY - padY) / handPadDis + padY);
  }

  // painter.drawText(200,200,tr("%1,%2,%3").arg(JoyStickX).arg(JoyStickY).arg(handPaddis));

  painter.setPen(Qt::NoPen);
  painter.setBrush(JoyStickColor);
  painter.drawEllipse(QPoint(JoyStickX, JoyStickY), JoyStickR,
                      JoyStickR);
}

void QJoyStick::mouseMoveEvent(QMouseEvent *event)
{
  static bool r = false;
  mouseX = event->pos().x();
  mouseY = event->pos().y();
  if (r == true) {
    update();
    r = false;
  } else {
    r = true;
  }
}

void QJoyStick::mouseReleaseEvent(QMouseEvent *event)
{
  mouseX = width() / 2;
  mouseY = height() / 2;
  tim->stop();
  mousePressed = false;
  emit keyNumChanged(AppEnums::QRobotAction::Stop);
  update();
}

void QJoyStick::mousePressEvent(QMouseEvent *event)
{
  mouseX = event->pos().x();
  mouseY = event->pos().y();
  tim->start(100);
  mousePressed = true;
  update();
}

double QJoyStick::Pointdis(int a, int b, int x, int y)
{
  return sqrt((double)((x - a) * (x - a) + (y - b) * (y - b)));
}

int QJoyStick::getKeyNum()
{
  int x, y;
  int keynum;
  x = (int)(JoyStickX * 3.0 / (padR * 2));
  y = (int)(JoyStickY * 3.0 / (padR * 2));
  keynum = 3 * y + x;
  return keynum;
}






