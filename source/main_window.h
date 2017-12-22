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

typedef double Real;
// typedef std::vector<std::vector<Real> > Samples;
typedef std::vector<Real> Samples;

namespace Ui {
class MainWindow;
}

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
  void addGraph1(Samples data, double start, double step, double yOffset, double yScale);
  void addGraph2(Samples data, double start, double step, double yOffset, double yScale);
  void updateGraph(int value);
  void contextMenuRequest(QPoint pos);
  void removeSelectedGraph();
  void removeAllGraphs();
  void mousePress();
  void mouseWheel();
  // void smooth();

private:;
  void load_csv(unsigned int type=1);
  Samples load_csv(QString filepath, double *xOffset, double *xScale, double *yOffset, double *yScale);

private:;
  Ui::MainWindow *ui;
  // Немодальные диалоги задания величины.
  QInputDialog *QIDRmeas; // Измерительного сопротивления.
  QInputDialog *QIDFreqParRes; // Частоты при параллельном резонансе.
  float Rmeas;
  float ParResFreq;
  // Paths to opened .csv files.
  QString path_to_radio_csv;
  QString path_to_attenuation_csv;
  Samples samples_radio;
  Samples samples_attenuation;
};

#endif
