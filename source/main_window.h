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
#include <complex>
#include "../custom_plot/qcustomplot.h"

typedef std::complex<float> Complex;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow { //, public Ui::MainWindow {
  Q_OBJECT

public:;
  explicit MainWindow(QWidget *parent=0);
  ~MainWindow();

private slots:;
  void open_image();
  void open_side_scan_proj();

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
  void load_image();
  void load_side_scan_project(QString bd_path, QString prj_name, QString track_name);

private:;
  Ui::MainWindow *ui;
  // Path to opened project or .png or .tiff image.
  QString path_to_data;
  // Data stored in easy to access way.
  QVector<QVector<float> > samples;
  QVector<QVector<float> > amplitude_samples;
  QVector<QVector<Complex> > quadrature_samples;
};

#endif
