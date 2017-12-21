#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <cmath>
#include <cstdlib>
#include <QtGui>
#include <QObject>
#include <QCursor>
#include <QPointF>
#include <QString>
#include <QFile>
#include <QByteArray>
#include <QMouseEvent>
#include <QGraphicsRectItem>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QGraphicsSceneMouseEvent>
#include <QImage>
#include "main_window.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
                       QMainWindow(parent),
                       ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  Rmeas = 1; // Om
  ResFreq = 1; // kHz

  ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                  QCP::iSelectLegend | QCP::iSelectPlottables);

  ui->customPlot->xAxis->setRange(-8, 8);
  ui->customPlot->yAxis->setRange(-5, 5);
  ui->customPlot->axisRect()->setupFullAxesBox();

  ui->customPlot->plotLayout()->insertRow(0);
  QCPTextElement *title = new QCPTextElement(ui->customPlot, "Scanline Viewer", QFont("sans", 17, QFont::Bold));
  ui->customPlot->plotLayout()->addElement(0, 0, title);

  ui->customPlot->xAxis->setLabel("Timestamp.");
  ui->customPlot->yAxis->setLabel("Signal, V.");
  ui->customPlot->legend->setVisible(true);
  QFont legendFont = font();
  legendFont.setPointSize(10);
  ui->customPlot->legend->setFont(legendFont);
  ui->customPlot->legend->setSelectedFont(legendFont);

  // Legend box shall not be selectable, only legend items.
  ui->customPlot->legend->setSelectableParts(QCPLegend::spItems);

  ui->customPlot->rescaleAxes();

  // Connect slot that ties some axis selections together (especially opposite axes):
  connect(ui->customPlot, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged()));
  // Connect slots that takes care that when an axis is selected, only that direction can be dragged and zoomed:
  connect(ui->customPlot, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress()));
  connect(ui->customPlot, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel()));

  // Make bottom and left axes transfer their ranges to top and right axes:
  connect(ui->customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->xAxis2, SLOT(setRange(QCPRange)));
  connect(ui->customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->yAxis2, SLOT(setRange(QCPRange)));

  // Connect some interaction slots:
  connect(ui->customPlot, SIGNAL(axisDoubleClick(QCPAxis*, QCPAxis::SelectablePart, QMouseEvent*)), this, SLOT(axisLabelDoubleClick(QCPAxis*, QCPAxis::SelectablePart)));
  connect(ui->customPlot, SIGNAL(legendDoubleClick(QCPLegend*, QCPAbstractLegendItem*, QMouseEvent*)), this, SLOT(legendDoubleClick(QCPLegend*, QCPAbstractLegendItem*)));
  connect(title, SIGNAL(doubleClicked(QMouseEvent*)), this, SLOT(titleDoubleClick(QMouseEvent*)));


  // Setup policy and connect slot for context menu popup:
  ui->customPlot->setContextMenuPolicy(Qt::CustomContextMenu);
  // connect(ui->customPlot, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));

  connect(ui->action_OpenRadioSignal, SIGNAL(triggered()), this, SLOT(open_csv_radio()));
  connect(ui->action_OpenAttenuationSignal, SIGNAL(triggered()), this, SLOT(open_csv_attenuation()));

  QIDRmeas=new QInputDialog(this);
  QIDFreqParRes=new QInputDialog(this);
}

MainWindow::~MainWindow()
{
  delete ui;
}

/**
 * Reaction on menu button click.
 */
void MainWindow::open_csv_radio() {
  path_to_radio_csv = QFileDialog::getOpenFileName(this, QObject::tr("Укажите путь к радиоимпульсу."), "/home/evgeny/workspace/hyscan5/signal-plotter/data", QObject::tr("(*.csv)"));

  if(!path_to_radio_csv.isEmpty ()) {
      load_csv(1);
  } else {
    QMessageBox::information(this, QObject::tr("SignalPlotter"),
                             QObject::tr("Файл не был открыт."));
  }
}

void MainWindow::open_csv_attenuation() {
  path_to_attenuation_csv = QFileDialog::getOpenFileName(this, QObject::tr("Укажите путь к затухающему сигналу."), "/home/evgeny/workspace/hyscan5/signal-plotter/data", QObject::tr("(*.csv)"));

  if(!path_to_attenuation_csv.isEmpty ()) {
    load_csv(2);
  } else {
    QMessageBox::information(this, QObject::tr("SignalPlotter"),
                             QObject::tr("Файл не был открыт."));
  }
}

/*
 * Wrapper under common .csv data loader.
 */
void MainWindow::load_csv(unsigned int type) {
  if (type == 1) {
    samples_radio.clear ();
    samples_radio = load_csv(path_to_radio_csv);
  } else if(type == 2) {
    samples_attenuation.clear();
    samples_attenuation = load_csv(path_to_attenuation_csv);
  } else {
    printf("Wrong signal type.\n");
  }
  addGraph(0);
}

/**
 * Physical data load.
 * Format of .csv file:
 * "X, CH2, Start, Increment,
 * Sequence, Volt,
 * 0, num,
 * 1, num,
 * 2, num,..."
 */
Samples MainWindow::load_csv(QString filepath) {
  Samples samples;
  QByteArray buf;

  // Open filename.
  QFile file(filepath);
  if(!file.open(QIODevice::ReadOnly)) {
    printf("File %s wasn't opened.\n", filepath.toAscii().data());
    return Samples();
  }

  // for debug.
  QFile file_d("out_debug.txt");
  if(!file_d.open(QIODevice::WriteOnly)) {
    printf("File %s wasn't opened.\n", QString("out_debug.txt").toAscii().data());
    return Samples();
  }


  // Skip headline.
  buf = file.readLine();
  // Read the second line
  buf = file.readLine();
  QList<QByteArray> list = buf.split(',');

  // Load points from .csv file.
  while (buf.size() > 2) {
	buf = file.readLine();
	if(buf.size() > 2) {
	  list = buf.split(',');
	  printf("size of list: %d, %s, %s, %s \n", list.size(), list[0].data(), list[1].data(), list[2].data());
	  printf("%f\n", list[1].toDouble());
	  file_d.write(list[1]);
	  file_d.write("\n");
	  samples.push_back(list[1].toDouble() * 10000);
	}
  }

  printf("Everything was red.\n");

  file.close();
  file_d.close();

  return samples;
}

/**
 * Add graph.
 */
void MainWindow::addGraph(int curPos)
{
  // Determine size of current plot. (number of points in graph).
  int curSize;
  curSize = this->samples_radio.size();

  double xScale = 1;
  // double yScale = 1;
  // X and Y offsets are always 0.
  double xOffset = 0;
  // double yOffset = 0;

  QVector<double> x(curSize);
  QVector<double> y(curSize);

  for (int i=0; i < curSize; i++)
  {
    x[i] = i * xScale + xOffset;
    y[i] = samples_radio[i];
  }

  ui->customPlot->addGraph();
  ui->customPlot->graph()->setName(QString("New graph %1").arg(ui->customPlot->graphCount() - 1));
  ui->customPlot->graph()->setData(x, y);
  ui->customPlot->graph()->setLineStyle(QCPGraph::lsImpulse);
  // if (rand() % 100 > 50)
    // ui->customPlot->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape)(rand() % 14 + 1)));
  QPen graphPen;
  // graphPen.setColor(QColor(rand() % 245 + 10, rand() % 245 + 10, rand() % 245 + 10));
  graphPen.setColor(QColor(250, 10, 5));
  graphPen.setWidthF(3); //(rand() / (double)RAND_MAX * 2 + 1);
  ui->customPlot->graph()->setPen(graphPen);
  ui->customPlot->replot();
}

void MainWindow::updateGraph(int value) {
  // Remove all graphs.
  removeAllGraphs();

  // Get current slider position.
  // int curPos = ui->horizontalSlider->sliderPosition();

  // Add graph related to current position.
  // addGraph(curPos);
}

void MainWindow::titleDoubleClick(QMouseEvent* event)
{
  Q_UNUSED(event)
  if (QCPTextElement *title = qobject_cast<QCPTextElement*>(sender()))
  {
    // Set the plot title by double clicking on it.
    bool ok;
    QString newTitle = QInputDialog::getText(this, "QCustomPlot example", "New plot title:", QLineEdit::Normal, title->text(), &ok);
    if (ok)
    {
      title->setText(newTitle);
      ui->customPlot->replot();
    }
  }
}

void MainWindow::axisLabelDoubleClick(QCPAxis *axis, QCPAxis::SelectablePart part)
{
  // Set an axis label by double clicking on it.
  // Only react when the actual axis label is clicked, not tick label or axis backbone.
  if (part == QCPAxis::spAxisLabel)
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

void MainWindow::legendDoubleClick(QCPLegend *legend, QCPAbstractLegendItem *item)
{
  // Rename a graph by double clicking on its legend item.
  Q_UNUSED(legend)
  // Only react if item was clicked (user could have clicked on border padding of legend where there is no item, then item is 0).
  if (item)
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

void MainWindow::selectionChanged()
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

  // Make top and bottom axes be selected synchronously, and handle axis and tick labels as one selectable object:
  if (ui->customPlot->xAxis->selectedParts().testFlag(QCPAxis::spAxis) || ui->customPlot->xAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
      ui->customPlot->xAxis2->selectedParts().testFlag(QCPAxis::spAxis) || ui->customPlot->xAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
  {
    ui->customPlot->xAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    ui->customPlot->xAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
  }
  // Make left and right axes be selected synchronously, and handle axis and tick labels as one selectable object:
  if (ui->customPlot->yAxis->selectedParts().testFlag(QCPAxis::spAxis) || ui->customPlot->yAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
      ui->customPlot->yAxis2->selectedParts().testFlag(QCPAxis::spAxis) || ui->customPlot->yAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
  {
    ui->customPlot->yAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    ui->customPlot->yAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
  }

  // Synchronize selection of graphs with selection of corresponding legend items:
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

void MainWindow::mouseWheel()
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

void MainWindow::mousePress()
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

void MainWindow::graphClicked(QCPAbstractPlottable *plottable, int dataIndex)
{
  // Since we know we only have QCPGraphs in the plot, we can immediately access interface1D()
  // usually it's better to first check whether interface1D() returns non-zero, and only then use it.
  double dataValue = plottable->interface1D()->dataMainValue(dataIndex);
  QString message = QString("Clicked on graph '%1' at data point #%2 with value %3.").arg(plottable->name()).arg(dataIndex).arg(dataValue);
  ui->statusBar->showMessage(message, 2500);
}

void MainWindow::removeSelectedGraph()
{
  if (ui->customPlot->selectedGraphs().size() > 0)
  {
    ui->customPlot->removeGraph(ui->customPlot->selectedGraphs().first());
    ui->customPlot->replot();
  }
}

void MainWindow::removeAllGraphs()
{
  ui->customPlot->clearGraphs();
  ui->customPlot->replot();
}

#endif
