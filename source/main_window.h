#ifndef __MAIN_WINDOW_H__
#define __MAIN_WINDOW_H__

#include <QWidget>
#include <QObject>
#include <QGraphicsScene>
#include <QMainWindow>
#include <QGraphicsView>
#include <QAction>
#include <QGraphicsItem>
#include <QInputDialog>
#include <stdio.h>
#include <iostream>
#include "../custom_plot/qcustomplot.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow { //, public Ui::MainWindow {
  Q_OBJECT

public:;
  explicit MainWindow(QWidget *parent=0);
  ~MainWindow();

private slots:;
  void titleDoubleClick(QMouseEvent *event);
  void axisLabelDoubleClick(QCPAxis* axis, QCPAxis::SelectablePart part);
  void legendDoubleClick(QCPLegend* legend, QCPAbstractLegendItem* item);
  void selectionChanged();
  void graphClicked(QCPAbstractPlottable *plottable, int dataIndex);
  void mousePress();
  void mouseWheel();

private:;
  Ui::MainWindow *ui;
};

#endif
