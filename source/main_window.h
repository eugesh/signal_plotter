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
#include <QVector>
#include <QString>
#include <QMap>
#include <complex>
#include <vector>
#include "../custom_plot/qcustomplot.h"

typedef std::complex<float> Complex;
typedef double Real;
typedef std::vector<std::vector<Real> > Samples;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

public:;
  explicit MainWindow(QWidget *parent=0);
  ~MainWindow();

private slots:;
  void add_acoustic_graph();
  void add_raw_data_re_graph();
  void add_raw_data_im_graph();
  void add_raw_data_ampl_graph();
  void add_raw_data_phase_graph();
  void add_re_spectrum_graph();
  void add_im_spectrum_graph();
  void add_ampl_spectrum_graph();
  void add_phase_spectrum_graph();

  void open_image();
  void open_side_scan_proj();

  void titleDoubleClick(QMouseEvent *event);
  void axisLabelDoubleClick(QCPAxis* axis, QCPAxis::SelectablePart part);
  void contextMenuRequest(QPoint pos);
  void legendDoubleClick(QCPLegend* legend, QCPAbstractLegendItem* item);
  void selectionChanged();
  void graphClicked(QCPAbstractPlottable *plottable, int dataIndex);
  void addGraph(int pos, QString type);
  void updateGraph(int value);
  void removeSelectedGraph();
  void removeAllGraphs();
  void mousePress();
  void mouseWheel();

private:;
  void createActions();
  /* Get samples by name:
   "RawDataRe", "RawDataIm", "RawDataAmpl", "Acoustic", "AmplSpectrum", "PhaseSpectrum", "PNG". */
  Samples get_samples(QString name);
  void load_image();
  void load_side_scan_project(QString bd_path, QString prj_name, QString track_name);
  void read_amplitude_values(QString prj_name, QString track_name);
  void read_quadrature_values(QString prj_name, QString track_name);
  // void addActiveGraphs();

private:;
  Ui::MainWindow *ui;
  // Path to opened project or .png or .tiff image.
  QString path_to_data;
  // List of active graphs. Allowable names:
  // "PNG", "Acoustic", "RawDataRe", "RawDataIm", "RawDataAmpl", "RawDataPhase", "AmplSpectrum", "PhaseSpectrum"
  QStringList activeGraphs;
  // Data stored in easy to access way. Key - name from activeGraphs. Value - data.
  QMap<QString, Samples> all_samples;
  // The greater value the sparser data you see.
  float decimation_factor;
};

#endif
