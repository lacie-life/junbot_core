#ifndef QJOYSTICK_H
#define QJOYSTICK_H

#include <QWidget>
#include <QDebug>
#include <QDrag>
#include <QMouseEvent>
#include <QPainter>
#include <QTimer>
#include <QWidget>
#include <QtMath>

class QJoyStick : public QWidget
{
  Q_OBJECT
public:
  QJoyStick(QWidget *parent = nullptr);
  ~QJoyStick();

signals:
  void keyNumChanged(int num);

protected:
  void paintEvent(QPaintEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
//  void resizeEvent(QResizeEvent *event) override;

private:
  int mouseX;
  int mouseY;
  int JoyStickX;
  int JoyStickY;
  int JoyStickR;
  int padX;  // chassis
  int padY;
  int padR;
  double handPadDis; // distance between the center of two circles
  bool mousePressed;
  QTimer *tim;

private:
  double Pointdis(int a, int b, int x, int y);  // two point distance
  int getKeyNum();
};

#endif // QJOYSTICK_H
