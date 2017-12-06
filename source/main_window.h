/*!
 * 2D Plot analyser. SignalPlotter.
 *
 * Plots 2D graph. Source data is
 * -raws of .png image may be:
 * -scanlines taken by side-scan echo-sounder.
 *
 */

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
#include <QVector>
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
  void open_csv_radio();
  void open_csv_attenuation();
  void titleDoubleClick(QMouseEvent *event);
  void axisLabelDoubleClick(QCPAxis* axis, QCPAxis::SelectablePart part);
  void legendDoubleClick(QCPLegend* legend, QCPAbstractLegendItem* item);
  void selectionChanged();
  void graphClicked(QCPAbstractPlottable *plottable, int dataIndex);
  void addGraph(int pos);
  void updateGraph(int value);
  void removeSelectedGraph();
  void removeAllGraphs();
  void mousePress();
  void mouseWheel();

private:;
  void load_csv(unsigned int type=1);
  QVector<QVector<float> > load_csv(QString filepath);

private:;
  Ui::MainWindow *ui;
  float Rmeas;
  float ResFreq;
  // Paths to opened .csv files.
  QString path_to_radio_csv;
  QString path_to_attenuation_csv;
  QVector<QVector<float> > samples_radio;
  QVector<QVector<float> > samples_attenuation;
};

#endif
