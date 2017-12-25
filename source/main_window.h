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
#include "signals_eval.h"

namespace Ui {
class MainWindow;
}


struct GraphParams {
 /**
  * Parameters of graphs in customPlot.
  */
  double xOffset;
  double xScale;
  double yOffset;
  double yScale;
};

class MainWindow : public QMainWindow {
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
  void addGraph1(Samples data, GraphParams const& graph_params);
  void addGraph2(Samples data, GraphParams const& graph_params);
  void updateGraph();
  void contextMenuRequest(QPoint pos);
  void removeSelectedGraph();
  void removeAllGraphs();
  void mousePress();
  void mouseWheel();
  void smooth();
  void approximate();
  void estimate_params();

private:;
  void load_csv(unsigned int type=1);
  Samples load_csv(QString filepath, GraphParams *graph_params);

private:;
  Ui::MainWindow *ui;
  // Немодальные диалоги задания величины.
  QInputDialog *QIDRmeas; // Измерительного сопротивления.
  QInputDialog *QIDFreqParRes; // Частоты при параллельном резонансе.
  float Rmeas;
  float ParResFreq;
  unsigned int median_mask;
  // Paths to opened .csv files.
  QString path_to_radio_csv;
  QString path_to_attenuation_csv;
  Samples samples_radio;
  Samples samples_attenuation;
  GraphParams graph_radio;
  GraphParams graph_attenuation;
};

#endif
