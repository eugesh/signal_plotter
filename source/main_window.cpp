#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <QtGui>
#include <QObject>
#include <QCursor>
#include <QPointF>
#include <QMouseEvent>
#include <QGraphicsRectItem>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QGraphicsSceneMouseEvent>
#include <QImage>
#include "main_window.h"
#include "ui_mainwindow.h"
#include "cpp-hyscan-db-wrap.h"
#include "cpp-hyscan-acoustic-data.h"
#include "cpp-hyscan-raw-data.h"


MainWindow::MainWindow(QWidget *parent) :
                       QMainWindow(parent),
                       ui(new Ui::MainWindow)
{
  decimation_factor = 16;
  ui->setupUi(this);

  ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                  QCP::iSelectLegend | QCP::iSelectPlottables);

  ui->customPlot->xAxis->setRange(0, 1000);
  ui->customPlot->yAxis->setRange(0, 255);
  ui->customPlot->axisRect()->setupFullAxesBox();

  ui->customPlot->plotLayout()->insertRow(0);
  QCPTextElement *title = new QCPTextElement(ui->customPlot, "Scanline Viewer", QFont("sans", 17, QFont::Bold));
  ui->customPlot->plotLayout()->addElement(0, 0, title);

  ui->customPlot->xAxis->setLabel("x Axis");
  ui->customPlot->yAxis->setLabel("y Axis");
  ui->customPlot->legend->setVisible(true);
  QFont legendFont = font();
  legendFont.setPointSize(10);
  ui->customPlot->legend->setFont(legendFont);
  ui->customPlot->legend->setSelectedFont(legendFont);
  ui->customPlot->legend->setSelectableParts(QCPLegend::spItems); // legend box shall not be selectable, only legend items

  ui->customPlot->rescaleAxes();

  // connect slot that ties some axis selections together (especially opposite axes):
  connect(ui->customPlot, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged()));
  // connect slots that takes care that when an axis is selected, only that direction can be dragged and zoomed:
  connect(ui->customPlot, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress()));
  connect(ui->customPlot, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel()));

  // make bottom and left axes transfer their ranges to top and right axes:
  connect(ui->customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->xAxis2, SLOT(setRange(QCPRange)));
  connect(ui->customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->yAxis2, SLOT(setRange(QCPRange)));

  // Connect some interaction slots:
  connect(ui->customPlot, SIGNAL(axisDoubleClick(QCPAxis*, QCPAxis::SelectablePart, QMouseEvent*)), this, SLOT(axisLabelDoubleClick(QCPAxis*, QCPAxis::SelectablePart)));
  connect(ui->customPlot, SIGNAL(legendDoubleClick(QCPLegend*, QCPAbstractLegendItem*, QMouseEvent*)), this, SLOT(legendDoubleClick(QCPLegend*, QCPAbstractLegendItem*)));
  connect(title, SIGNAL(doubleClicked(QMouseEvent*)), this, SLOT(titleDoubleClick(QMouseEvent*)));

  // connect slot that shows a message in the status bar when a graph is clicked:
  connect(ui->customPlot, SIGNAL(plottableClick(QCPAbstractPlottable*, int, QMouseEvent*)), this, SLOT(graphClicked(QCPAbstractPlottable*, int)));

  // setup policy and connect slot for context menu popup:
  ui->customPlot->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(ui->customPlot, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));

  connect(ui->action_OpenImage, SIGNAL(triggered()), this, SLOT(open_image()));
  connect(ui->actionLoad_Project, SIGNAL(triggered()), this, SLOT(open_side_scan_proj()));

  createActions();

  connect(ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(updateGraph(int)));
}

void MainWindow::createActions() {
  connect(ui->action_Acoustic, SIGNAL(triggered()), this, SLOT(add_acoustic_graph()));
  connect(ui->action_Re, SIGNAL(triggered()), this, SLOT(add_raw_data_re_graph()));
  connect(ui->action_Im, SIGNAL(triggered()), this, SLOT(add_raw_data_im_graph()));
  connect(ui->action_Ampl, SIGNAL(triggered()), this, SLOT(add_raw_data_ampl_graph()));
  connect(ui->action_Phase, SIGNAL(triggered()), this, SLOT(add_raw_data_phase_graph()));
  connect(ui->action_PhaseSpectrum, SIGNAL(triggered()), this, SLOT(add_phase_spectrum_graph()));
  connect(ui->action_AmplSpectrum, SIGNAL(triggered()), this, SLOT(add_ampl_spectrum_graph()));
}

MainWindow::~MainWindow()
{
  delete ui;
}

Samples
MainWindow::get_samples(QString name) {
  return all_samples[name];
}

/**
 * Reaction on button click.
 */
void
MainWindow::open_image() {
  path_to_data = QFileDialog::getOpenFileName(this, tr("Укажите путь к изображению"), "/home/evgeny/data", tr("Images (*.png *.xpm *.jpg *.bmp *.jpeg)"));

  if(!path_to_data.isEmpty ()) {
      load_image();
  } else {
    QMessageBox::information(this, tr("SignalPlotter"),
           tr("Файл не был открыт."));
           // tr("Файл не был открыт %1.").arg(path_to_data));
  }
}

/**
 * Physical data load.
 */
void
MainWindow::load_image() {
  Samples samples;
  // It is loaded from open_image slot so it is always image.
  QImage img = QImage(path_to_data);

  samples.clear ();

  // Fill out samples.
  for (int i = 0; i < img.height(); ++i ) {
    std::vector<Real> scan_line;
    for (int j =0 ; j < img.width(); ++j) {
        scan_line.push_back((float) qGray(img.pixel(j, i)));
    }
    samples.push_back(scan_line);
  }

  // ui->horizontalSlider->setMinimum(0);
  // Set maximum sliding value via the first line.
  // ui->horizontalSlider->setMaximum(samples[0].size());
  ui->horizontalSlider->setRange(0, samples.size() - 1);
  ui->horizontalSlider->setValue(0);
  all_samples.insert("PNG", samples);
  activeGraphs.append("PNG");
  addGraph(0, "PNG");
}

/**
 * Reaction on button click.
 */
void
MainWindow::open_side_scan_proj() {
  /* Get track path. */
  QString track_path = QFileDialog::getExistingDirectory (this, tr("Укажите путь к галсу. Open track."), "/home/evgeny/data/line");

  if(!track_path.isEmpty()) {
    QStringList path_words = track_path.split(QRegExp("[/\\\\]"));
    QString track_name = path_words.last();
    std::cout << "track_name = " << qPrintable(track_name) << std::endl;

    QString project_dir_name = path_words.at(path_words.size() - 2);
    std::cout << "project_dir_name = " << qPrintable(project_dir_name) << std::endl;

    QString bd_path;

    for (int i = 1; i < path_words.size() - 2; ++i)
        bd_path += QString("/%1").arg(path_words[i]);
    std::cout << "bd_path = " << qPrintable(bd_path) << std::endl;

    load_side_scan_project(bd_path, project_dir_name, track_name);
  } else {
    QMessageBox::information(this, tr("SignalPlotter"),
           tr("Файл не был открыт."));
  }
}

/**
 * Physical data load.
 */
void
MainWindow::load_side_scan_project(QString bd_path, QString prj_name, QString track_name) {
  // Remove previous data
  removeAllGraphs();
  all_samples.clear();
  ui->horizontalSlider->setValue(0);
  // Connect/Reconnect to DB.
  QString uri = QString ("file://%1").arg(bd_path);
  int bd_check = connect_to_bd (uri.toAscii().data());

  // Open project.
  // unsigned int project_id = open_project(prj_name.toAscii().data());

  read_amplitude_values(prj_name, track_name);

  activeGraphs.append("Acoustic");
  addGraph(0, "Acoustic");

  read_quadrature_values(prj_name, track_name);

  activeGraphs.append("RawDataRe");
  activeGraphs.append("RawDataIm");
  addGraph(0, "RawDataRe");
  addGraph(0, "RawDataIm");
}

void
MainWindow::read_amplitude_values(QString prj_name, QString track_name) {
  unsigned int values_count = 0;
  // Track id.
  unsigned int ad_id = acoustic_data_new (prj_name.toAscii().data(), track_name.toAscii().data(), 101, 1);

  unsigned int first_index = acoustic_data_get_first_index_in_range (ad_id);
  unsigned int last_index = acoustic_data_get_last_index_in_range (ad_id);
  unsigned int range = last_index - first_index;

  printf("ad_id = %d, fi = %d, li = %d, r = %d\n", ad_id, first_index, last_index, range);

  for (int i = first_index; i < last_index; i += 1) {
      values_count = acoustic_data_get_values_count(ad_id, i);
      printf("values_count acoustic = %d", values_count);
      float *ampl_buffer = new float[values_count];
      values_count = acoustic_data_get_values(ad_id, i, ampl_buffer, values_count, NULL);
      std::vector<Real> vec_ampl_scanline(values_count);

      // vec_ampl_scanline.data() = ampl_buffer;
      for (int j = 0; j < values_count; j += decimation_factor) {
          vec_ampl_scanline.push_back(ampl_buffer[j]);
      }
      // amplitude_samples.append(vec_ampl_scanline);
      all_samples["Acoustic"].push_back(vec_ampl_scanline);
      delete[] ampl_buffer;
  }

  ui->customPlot->xAxis->setRange(0, 2 * values_count);
  ui->customPlot->yAxis->setRange(-1, 1);
}

void
MainWindow::read_quadrature_values(QString prj_name, QString track_name) {
  unsigned int values_count = 0;
  // Track id.
  unsigned int rd_id = raw_data_new (prj_name.toAscii().data(), track_name.toAscii().data(), 101, 1);
  unsigned int first_index = raw_data_get_first_index_in_range (rd_id);
  unsigned int last_index = raw_data_get_last_index_in_range (rd_id);
  unsigned int range = last_index - first_index;

  printf("rd_id = %d, fi = %d, li = %d, r = %d\n", rd_id, first_index, last_index, range);

  for (int i = first_index; i < last_index; i += 1) {
      values_count = raw_data_get_values_count(rd_id, i);
      printf("values_count re, im = %d", values_count);
      ComplexFloat *buffer = new ComplexFloat[values_count];

      values_count = get_quadrature_values(rd_id, i, buffer, values_count, NULL);

      std::vector<Real> vec_val_scanline(values_count);

      // Take Re values.
      for (int j = 0; j < values_count; j += decimation_factor) {
          vec_val_scanline.push_back(buffer[j].re);
      }

      all_samples["RawDataRe"].push_back(vec_val_scanline);

      vec_val_scanline.clear();

      // Take Im values.
      for (int j = 0; j < values_count; j += decimation_factor) {
          vec_val_scanline.push_back(buffer[j].im);
      }

      all_samples["RawDataIm"].push_back(vec_val_scanline);

      delete[] buffer;
  }

  ui->customPlot->xAxis->setRange(0, 2 * values_count);
  ui->customPlot->yAxis->setRange(-1, 1);
}

/**
 * Add graph.
 *
 * \param curPos index of current line to view.
 */
void
MainWindow::addGraph(int curPos, QString type)
{
  // Determine size of current plot. (number of points in graph).
  int curSize;
  Samples samples = get_samples(type);
  if (curPos < samples.size())
    curSize = samples[curPos].size();
  else {
    std::cout << "Error MainWindow::addGraph: invalid slide window position." << std::endl;
    return ;
  }

  double xScale = 1;
  double yScale = 1;
  // X and Y offsets are always 0.
  double xOffset = 0;
  double yOffset = 0;

  QVector<double> x(curSize);
  QVector<double> y(curSize);

  for (int i=0; i < curSize; i++)
  {
    x[i] = i;
    y[i] = samples[curPos][i];
    // memcpy(y.data(), (double*)(samples[curPos].data()), curSize * sizeof(double));
  }

  ui->customPlot->addGraph();
  ui->customPlot->graph()->setName(type); //.arg(ui->customPlot->graphCount() - 1));
  ui->customPlot->graph()->setData(x, y);
  ui->customPlot->graph()->setLineStyle(QCPGraph::lsLine); //lsImpulse);
  // if (rand() % 100 > 50)
    // ui->customPlot->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape)(rand() % 14 + 1)));
  QPen graphPen;
  graphPen.setColor(QColor(rand() % 245 + 10, rand() % 245 + 10, rand() % 245 + 10));
  // graphPen.setColor(QColor(250, 10, 5));
  graphPen.setWidthF(3); //(rand() / (double)RAND_MAX * 2 + 1);
  ui->customPlot->graph()->setPen(graphPen);
  ui->customPlot->replot();
  ui->horizontalSlider->setRange(0, samples.size() - 1);
}

void
MainWindow::updateGraph(int value) {
  // Remove all graphs.
  removeAllGraphs();

  // Get current slider position.
  int curPos = ui->horizontalSlider->sliderPosition();

  // Add each active graph related to current position.
  foreach(const QString &str, activeGraphs) {
    addGraph(curPos, str);
  };
}

void
MainWindow::titleDoubleClick(QMouseEvent* event)
{
  Q_UNUSED(event)
  if (QCPTextElement *title = qobject_cast<QCPTextElement*>(sender()))
  {
    // Set the plot title by double clicking on it
    bool ok;
    QString newTitle = QInputDialog::getText(this, "QCustomPlot example", "New plot title:", QLineEdit::Normal, title->text(), &ok);
    if (ok)
    {
      title->setText(newTitle);
      ui->customPlot->replot();
    }
  }
}

void
MainWindow::axisLabelDoubleClick(QCPAxis *axis, QCPAxis::SelectablePart part)
{
  // Set an axis label by double clicking on it
  if (part == QCPAxis::spAxisLabel) // only react when the actual axis label is clicked, not tick label or axis backbone
  {
    bool ok;
    QString newLabel = QInputDialog::getText(this, "QCustomPlot example", "New axis label:", QLineEdit::Normal, axis->label(), &ok);
    if (ok)
    {
      axis->setLabel(newLabel);
      ui->customPlot->replot();
    }
  }
}

void
MainWindow::legendDoubleClick(QCPLegend *legend, QCPAbstractLegendItem *item)
{
  // Rename a graph by double clicking on its legend item
  Q_UNUSED(legend)
  if (item) // Only react if item was clicked (user could have clicked on border padding of legend where there is no item, then item is 0)
  {
    QCPPlottableLegendItem *plItem = qobject_cast<QCPPlottableLegendItem*>(item);
    bool ok;
    QString newName = QInputDialog::getText(this, "QCustomPlot example", "New graph name:", QLineEdit::Normal, plItem->plottable()->name(), &ok);
    if (ok)
    {
      plItem->plottable()->setName(newName);
      ui->customPlot->replot();
    }
  }
}

void
MainWindow::selectionChanged()
{
  /*
   Normally, axis base line, axis tick labels and axis labels are selectable separately, but we want
   the user only to be able to select the axis as a whole, so we tie the selected states of the tick labels
   and the axis base line together. However, the axis label shall be selectable individually.

   The selection state of the left and right axes shall be synchronized as well as the state of the
   bottom and top axes.

   Further, we want to synchronize the selection of the graphs with the selection state of the respective
   legend item belonging to that graph. So the user can select a graph by either clicking on the graph itself
   or on its legend item.
  */

  // make top and bottom axes be selected synchronously, and handle axis and tick labels as one selectable object:
  if (ui->customPlot->xAxis->selectedParts().testFlag(QCPAxis::spAxis) || ui->customPlot->xAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
      ui->customPlot->xAxis2->selectedParts().testFlag(QCPAxis::spAxis) || ui->customPlot->xAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
  {
    ui->customPlot->xAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    ui->customPlot->xAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
  }
  // make left and right axes be selected synchronously, and handle axis and tick labels as one selectable object:
  if (ui->customPlot->yAxis->selectedParts().testFlag(QCPAxis::spAxis) || ui->customPlot->yAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
      ui->customPlot->yAxis2->selectedParts().testFlag(QCPAxis::spAxis) || ui->customPlot->yAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
  {
    ui->customPlot->yAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    ui->customPlot->yAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
  }

  // synchronize selection of graphs with selection of corresponding legend items:
  for (int i=0; i<ui->customPlot->graphCount(); ++i)
  {
    QCPGraph *graph = ui->customPlot->graph(i);
    QCPPlottableLegendItem *item = ui->customPlot->legend->itemWithPlottable(graph);
    if (item->selected() || graph->selected())
    {
      item->setSelected(true);
      graph->setSelection(QCPDataSelection(graph->data()->dataRange()));
    }
  }
}

void
MainWindow::mouseWheel()
{
  // If an axis is selected, only allow the direction of that axis to be zoomed.
  // If no axis is selected, both directions may be zoomed.

  if (ui->customPlot->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
    ui->customPlot->axisRect()->setRangeZoom(ui->customPlot->xAxis->orientation());
  else if (ui->customPlot->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
    ui->customPlot->axisRect()->setRangeZoom(ui->customPlot->yAxis->orientation());
  else
    ui->customPlot->axisRect()->setRangeZoom(Qt::Horizontal|Qt::Vertical);
}

void
MainWindow::mousePress()
{
  // If an axis is selected, only allow the direction of that axis to be dragged.
  // If no axis is selected, both directions may be dragged.

  if (ui->customPlot->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
    ui->customPlot->axisRect()->setRangeDrag(ui->customPlot->xAxis->orientation());
  else if (ui->customPlot->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
    ui->customPlot->axisRect()->setRangeDrag(ui->customPlot->yAxis->orientation());
  else
    ui->customPlot->axisRect()->setRangeDrag(Qt::Horizontal|Qt::Vertical);
}

void
MainWindow::contextMenuRequest(QPoint pos)
{
  QMenu *menu = new QMenu(this);
  menu->setAttribute(Qt::WA_DeleteOnClose);

  {
    if (ui->customPlot->selectedGraphs().size() > 0)
      menu->addAction("Remove selected graph", this, SLOT(removeSelectedGraph()));
    if (ui->customPlot->graphCount() > 0)
      menu->addAction("Remove all graphs", this, SLOT(removeAllGraphs()));
  }

  menu->popup(ui->customPlot->mapToGlobal(pos));
}

void
MainWindow::graphClicked(QCPAbstractPlottable *plottable, int dataIndex)
{
  // Since we know we only have QCPGraphs in the plot, we can immediately access interface1D()
  // usually it's better to first check whether interface1D() returns non-zero, and only then use it.
  double dataValue = plottable->interface1D()->dataMainValue(dataIndex);
  QString message = QString("Clicked on graph '%1' at data point #%2 with value %3.").arg(plottable->name()).arg(dataIndex).arg(dataValue);
  ui->statusBar->showMessage(message, 2500);
}

void
MainWindow::removeSelectedGraph()
{
  if (ui->customPlot->selectedGraphs().size() > 0)
  {
    activeGraphs.removeOne(ui->customPlot->graph()->name());
    ui->customPlot->removeGraph(ui->customPlot->selectedGraphs().first());
    ui->customPlot->replot();
  }
}

void
MainWindow::removeAllGraphs()
{
  ui->customPlot->clearGraphs();
  ui->customPlot->replot();
}

void MainWindow::add_acoustic_graph() {
  /* Check if acoustic data is loaded. */
  if(all_samples.contains("Acoustic")) {
      /* Set the graph as active. */
      activeGraphs.append("Acoustic");
  }

  /* Add graph to view area. */
  addGraph(ui->horizontalSlider->value(), "Acoustic");
}

void MainWindow::add_raw_data_re_graph() {
  /* Check if acoustic data is loaded. */
  if(all_samples.contains("RawDataRe")) {
      /* Set the graph as active. */
      activeGraphs.append("RawDataRe");
  }

  /* Add graph to view area. */
  addGraph(ui->horizontalSlider->value(), "RawDataRe");
}

void MainWindow::add_raw_data_im_graph() {
  /* Check if acoustic data is loaded. */
  if(all_samples.contains("RawDataIm")) {
      /* Set the graph as active. */
      activeGraphs.append("RawDataIm");
  }

  /* Add graph to view area. */
  addGraph(ui->horizontalSlider->value(), "RawDataIm");
}

void MainWindow::add_raw_data_ampl_graph() {

}

void MainWindow::add_raw_data_phase_graph() {

}

void MainWindow::add_re_spectrum_graph() {

}

void MainWindow::add_im_spectrum_graph() {

}

void MainWindow::add_ampl_spectrum_graph() {

}

void MainWindow::add_phase_spectrum_graph() {

}

/**
 * Private section.
 */


#endif
