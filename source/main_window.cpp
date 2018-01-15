#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <cmath>
#include <cstdlib>
#include <limits>
#include <cfloat>
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
#include "median.h"
#include "median_cpp.h"
#include "signals_eval.h"

const static double eps = 1e-10;

MainWindow::MainWindow(QWidget *parent) :
                       QMainWindow(parent),
                       ui(new Ui::MainWindow)
{
	ui->setupUi(this);
	Rmeas = 1; // Om
	ParResFreq = 1; // kHz
	end_index = 0;
	median_mask_size = 199;
	samples_radio_show = false;
	samples_radio_smoothed_show = false;
	samples_attenuation_show = false;
	samples_attenuation_smoothed_show = false;


	ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
																QCP::iSelectLegend | QCP::iSelectPlottables);

	ui->customPlot->xAxis->setRange(2e-5, 6e-5);
	ui->customPlot->yAxis->setRange(-2, 2);
	ui->customPlot->yAxis2->setRange(-0.025, 0.025);
	// ui->customPlot->yAxis->setScaleType(QCPAxis::stLogarithmic);
	ui->customPlot->axisRect()->setupFullAxesBox();

	ui->customPlot->plotLayout()->insertRow(0);
	QCPTextElement *title = new QCPTextElement(ui->customPlot, "Scanline Viewer", QFont("sans", 17, QFont::Bold));
	ui->customPlot->plotLayout()->addElement(0, 0, title);

	ui->customPlot->xAxis->setLabel(QObject::tr("Время, сек."));
	ui->customPlot->yAxis->setLabel(QObject::tr("Радиовоздействие, В."));
	ui->customPlot->yAxis2->setLabel(QObject::tr("Затухание, мВ."));
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
	connect(ui->customPlot, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));

	connect(ui->action_OpenRadioSignal, SIGNAL(triggered()), this, SLOT(open_csv_radio_dialog()));
	connect(ui->action_OpenAttenuationSignal, SIGNAL(triggered()), this, SLOT(open_csv_attenuation_dialog()));
	connect(ui->action_save_report, SIGNAL(triggered()), this, SLOT(save_report_dialog()));

	connect(ui->action_smooth, SIGNAL(triggered()), this, SLOT(smooth()));
	connect(ui->action_estim_param, SIGNAL(triggered()), this, SLOT(estimate_contour_params()));

	QIDRmeas = new QInputDialog(this);
	QIDFreqParRes = new QInputDialog(this);
	QIDNPeaks = new QInputDialog(this); // Число анализируемых пиков.
}

MainWindow::~MainWindow()
{
    delete ui;
}

/**
 * Reaction on menu button click.
 */
void MainWindow::open_csv_radio_dialog() {
	path_to_radio_csv = QFileDialog::getOpenFileName(this, QObject::tr("Укажите путь к радиоимпульсу."), "/home/evgeny/workspace/hyscan5/signal-plotter/data", QObject::tr("(*.csv)"));

	if(!path_to_radio_csv.isEmpty ()) {
		load_csv_radio();
	}
	else {
		QMessageBox::information(this, QObject::tr("SignalPlotter"),
														 QObject::tr("Файл не был открыт."));
	}
}

void MainWindow::open_csv_attenuation_dialog() {
	path_to_attenuation_csv = QFileDialog::getOpenFileName(this, QObject::tr("Укажите путь к затухающему сигналу."), "/home/evgeny/workspace/hyscan5/signal-plotter/data", QObject::tr("(*.csv)"));

	if(!path_to_attenuation_csv.isEmpty()) {
		load_csv_attenuation();
	} else {
		QMessageBox::information(this, QObject::tr("SignalPlotter"),
														 QObject::tr("Файл не был открыт."));
	}
}

void MainWindow::save_report_dialog() {
  path_to_report =
    QFileDialog::getSaveFileName(this, QObject::tr("Укажите путь для сохранения."),
                                 "/home/evgeny/workspace/hyscan5/signal-plotter/data", QObject::tr("(*.txt)"));

  if(!path_to_report.isEmpty ()) {
    save_report(path_to_report);
  }
  else {
    QMessageBox::information(this, QObject::tr("SignalPlotter"),
                             QObject::tr("Путь не был указан корректно."));
  }
}

void
MainWindow::save_report(QString filepath) {

}

/*
 * Wrapper under common .csv data loader.
 */
void MainWindow::load_csv_radio() {
	GraphParams g_params;

	samples_radio.clear();
	samples_radio = load_csv(path_to_radio_csv, &g_params);
	graph_radio = g_params;

	end_index = find_radio_signal_termination(samples_radio);

	samples_radio.erase(samples_radio.begin() + end_index, samples_radio.end());

	addGraph1(samples_radio, g_params);
}

void MainWindow::load_csv_attenuation() {
	GraphParams g_params;
	samples_attenuation.clear();
	samples_attenuation = load_csv(path_to_attenuation_csv, &g_params);
	graph_attenuation = g_params;

	addGraph2(samples_attenuation, g_params);
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
Samples MainWindow::load_csv(QString filepath, GraphParams *g_params) {
	Samples samples;
	QByteArray buf;

	// Open filename.
	QFile file(filepath);
	if(!file.open(QIODevice::ReadOnly)) {
		printf("File %s wasn't opened.\n", filepath.toAscii().data());
		return Samples();
	}

	// Skip headline.
	buf = file.readLine();
	// Read the second line.
	buf = file.readLine();
	QList<QByteArray> list = buf.split(',');

	// Set offsets and scales for each axis.
	g_params->xOffset = list[2].toDouble();
	g_params->xScale = list[3].toDouble();
	g_params->yOffset = 0;

	// Load points from .csv file.
	while (buf.size() > 2) {
		buf = file.readLine();
		if(buf.size() > 2) {
			list = buf.split(',');
			samples.push_back(list[1].toDouble());
		}
	}

	file.close();

	// Find min max.
	double min = 10000;
	double max = -10000;
	for (unsigned int i = 0; i < samples.size(); ++i){
		if(min > samples[i])
			min = samples[i];
		if(max < samples[i])
			max = samples[i];
	}
	g_params->yScale = (max - min) < eps ? 1 : 1 / (max - min);

	return samples;
}

/**
 * Add graph to left hand side Y scale.
 */
void MainWindow::addGraph1(Samples data, GraphParams const& g_params) {
	// Determine size of current plot. (number of points in graph).
	int curSize;
	curSize = data.size();

	QVector<double> x(curSize);
	QVector<double> y(curSize);

	printf("xScale, xOffset = %f %f\n", g_params.xScale, g_params.xOffset);

	for (int i=0; i < curSize; i++)
	{
		x[i] = i * g_params.xScale + g_params.xOffset;
		y[i] = data[i] * g_params.yScale + g_params.yOffset;
	}
	printf("xs = %.10f, xo = %.10f, ys = %.10f, yo = %.10f\n", g_params.xScale, g_params.xOffset, g_params.yScale, g_params.yOffset);

	ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis);
	ui->customPlot->graph()->setName(QString("New graph %1").arg(ui->customPlot->graphCount() - 1));
	ui->customPlot->graph()->setData(x, y);
	ui->customPlot->graph()->setLineStyle(QCPGraph::lsLine);
	QPen graphPen;
	graphPen.setColor(QColor(rand() % 245 + 10, rand() % 245 + 10, rand() % 245 + 10));
	graphPen.setWidthF(1);
	ui->customPlot->graph()->setPen(graphPen);
	ui->customPlot->replot();
}

/**
 * Add graph to right hand side Y scale.
 */
void MainWindow::addGraph2(Samples data, GraphParams const& g_params) {
	// Determine size of current plot. (number of points in graph).
	int curSize;
	curSize = data.size();

	QVector<double> x(curSize);
	QVector<double> y(curSize);

	printf("xScale, xOffset = %f %f\n", g_params.xScale, g_params.xOffset);

	for (int i=0; i < curSize; i++)
	{
		x[i] = i * g_params.xScale + g_params.xOffset;
		y[i] = data[i] * g_params.yScale + g_params.yOffset;
	}
	printf("xs = %.10f, xo = %.10f, ys = %.10f, yo = %.10f\n", g_params.xScale, g_params.xOffset, g_params.yScale, g_params.yOffset);

	// Attenuation graph has different y scale.
	ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis2);
	ui->customPlot->graph()->setName(QString("New graph %1").arg(ui->customPlot->graphCount() - 1));
	ui->customPlot->graph()->setData(x, y);
	ui->customPlot->graph()->setLineStyle(QCPGraph::lsLine);
	QPen graphPen;
	graphPen.setColor(QColor(rand() % 245 + 10, rand() % 245 + 10, rand() % 245 + 10));
	graphPen.setWidthF(1);
	ui->customPlot->graph()->setPen(graphPen);
	ui->customPlot->replot();
}

void MainWindow::updateGraph() {
  // Remove all graphs.
  removeAllGraphs();

  // Add graphs.
  if(!samples_radio.empty())
    addGraph1(samples_radio, graph_radio);
  if(!samples_attenuation.empty())
    addGraph2(samples_attenuation, graph_attenuation);
  if(!samples_radio_smoothed.empty())
    addGraph1(samples_radio_smoothed, graph_radio);
  if(!samples_attenuation_smoothed.empty())
    addGraph2(samples_attenuation_smoothed, graph_attenuation);
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

  if (ui->customPlot->yAxis->selectedParts().testFlag(QCPAxis::spAxis) || ui->customPlot->yAxis->selectedParts().testFlag(QCPAxis::spTickLabels))
  {
    ui->customPlot->yAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
  }

  if(ui->customPlot->yAxis2->selectedParts().testFlag(QCPAxis::spAxis) || ui->customPlot->yAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
  {
    ui->customPlot->yAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
  }

  // Synchronize selection of graphs with selection of corresponding legend items:
  for (int i=0; i < ui->customPlot->graphCount(); ++i)
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
  else if (ui->customPlot->yAxis2->selectedParts().testFlag(QCPAxis::spAxis))
    ui->customPlot->axisRect()->setRangeZoom(ui->customPlot->yAxis2->orientation());
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
  else if (ui->customPlot->yAxis2->selectedParts().testFlag(QCPAxis::spAxis))
    ui->customPlot->axisRect()->setRangeDrag(ui->customPlot->yAxis2->orientation());
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

void
MainWindow::contextMenuRequest(QPoint pos)
{
  QMenu *menu = new QMenu(this);
  menu->setAttribute(Qt::WA_DeleteOnClose);

  if (ui->customPlot->selectedGraphs().size() > 0)
    menu->addAction("Show selected graph", this, SLOT(showSelectedGraph()));
  if (ui->customPlot->selectedGraphs().size() > 0)
    menu->addAction("Hide selected graph", this, SLOT(hideSelectedGraph()));
  if (ui->customPlot->selectedGraphs().size() > 0)
    menu->addAction("Remove selected graph", this, SLOT(removeSelectedGraph()));
  if (ui->customPlot->graphCount() > 0)
    menu->addAction("Remove all graphs", this, SLOT(removeAllGraphs()));

  menu->popup(ui->customPlot->mapToGlobal(pos));
}

void
MainWindow::showSelectedGraph()
{
  if (ui->customPlot->selectedGraphs().size() > 0)
  {
    ui->customPlot->selectedGraphs().first()->setVisible(true);
    ui->customPlot->replot();
  }
}

void
MainWindow::hideSelectedGraph()
{
  if (ui->customPlot->selectedGraphs().size() > 0)
  {
    ui->customPlot->selectedGraphs().first()->setVisible(false);;
    ui->customPlot->replot();
  }
}

void
MainWindow::removeSelectedGraph()
{
  if (ui->customPlot->selectedGraphs().size() > 0)
  {
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

void
MainWindow::smooth() {
	// Smooth curves.

	if (samples_radio_smoothed.empty()) {
		samples_radio_smoothed = samples_radio;
		median1d(samples_radio_smoothed, samples_radio, median_mask_size);
		updateGraph();
	}

	if(samples_attenuation_smoothed.empty()) {
		samples_attenuation_smoothed = samples_attenuation;
		median1d(samples_attenuation_smoothed, samples_attenuation, median_mask_size * 3);
		updateGraph();
	}
}

void
MainWindow::approximate() {

}

void
MainWindow::estimate_contour_params() {
	double Q, f_r, f_a, Ra, La, Ca, Co;
	// Cut noisy signal endings after median filtering.
	Samples attenuation_signal = Samples(samples_attenuation_smoothed.begin() + 3 * median_mask_size, samples_attenuation_smoothed.end() - 3 * median_mask_size);
	Samples exp_curve, exp_curve_neg;

	// addGraph2(attenuation_signal, graph_attenuation); // debug

	double a, b;
	// signal_analyzer(&a, &b, attenuation_signal, &Q, &f_a, graph_attenuation.xOffset, graph_attenuation.xScale);
	signal_analyzer(&a, &b, &Q, &f_a);

	for(unsigned int i = end_index; i < samples_attenuation_smoothed.size(); ++i) {
		exp_curve.push_back(exp(a * i + b));
		exp_curve_neg.push_back(-exp(a * i + b));
	}

	GraphParams graph_exp = graph_attenuation;
	graph_exp.xOffset = graph_attenuation.xOffset + end_index * graph_attenuation.xScale;

	addGraph2(exp_curve, graph_exp); // debug
	addGraph2(exp_curve_neg, graph_exp); // debug

	// f_a /= graph_attenuation.xScale;

	printf("f_a = %f\nQ = %f\n", f_a, Q);
}

/**
 * Interface function for signal analysis.
 */
void
MainWindow::signal_analyzer(double *a, double *b, double *q_factor, double *freq) {
	Intervals zero_intervals = find_all_zeros_indices(samples_attenuation_smoothed);

	Peaks all_peaks = find_all_peaks(samples_attenuation_smoothed, zero_intervals);

	Peaks real_peaks = find_real_peaks(samples_attenuation_smoothed, all_peaks, 0.05);

	*freq = estimate_frequency(real_peaks, graph_attenuation.xOffset, graph_attenuation.xScale);

	*q_factor = estimate_quality(samples_attenuation_smoothed, real_peaks);
	estimate_quality_ls(a, b, samples_attenuation_smoothed, real_peaks, graph_attenuation.xOffset, graph_attenuation.xScale, end_index);

	double d = - *a / *freq;

	*q_factor = M_PI / d;

	printf("a = %f, f = %f, d = %.12f, Q = %f\n", *a, *freq, d, *q_factor);
}

#endif

/*void
MainWindow::smooth() {
    // Smooth curves.
    Samples samples_copy;

    if (!samples_radio.empty()) {
        samples_copy = Samples(samples_radio);
        median1d(samples_radio, samples_copy, median_mask_size);
        updateGraph();
    }

    if(!samples_attenuation.empty()) {
        samples_copy = Samples(samples_attenuation);
        median1d(samples_attenuation, samples_copy, median_mask_size * 3);
        updateGraph();
    }
}*/
