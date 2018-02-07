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
#include <QStringList>
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
#include "lowpass.h"
#include "signals_eval.h"

const static double eps = 1e-12;

QString path_from_fullname(QString const& fullpath) {
  QString filePath;

  QRegExp csv_regexp(".csv");
  QString fullPathBuf = fullpath;
  fullPathBuf.remove(csv_regexp);
  QString name = fullPathBuf.split('/').back();

  filePath = fullPathBuf.remove(name);

  std::cout << "filePath: " << qPrintable(filePath) << std::endl;

  return filePath;
}

template<typename T>
T
find_max(std::vector<T> vec) {
	T max = vec[0];
	for(unsigned int i = 0; i < vec.size(); ++i) {
		if(max < vec[i])
			max = vec[i];
	}

	return max;
}

template<typename T>
T
find_min(std::vector<T> vec) {
	T min = vec[0];
	for(unsigned int i = 0; i < vec.size(); ++i) {
		if(min > vec[i])
			min = vec[i];
	}

	return min;
}

template<typename T>
T
find_mean(std::vector<T> vec) {
  T mean = 0;
  for(unsigned int i = 0; i < vec.size(); ++i) {
      mean += vec[i];
  }

  return mean / vec.size();
}


MainWindow::MainWindow(QWidget *parent) :
                       QMainWindow(parent),
                       ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  U_max = I_max = 0;
  Rmeas = 2.55; // Om
  FreqParRes = 600; // kHz
  FreqNominalAntenna = 600; // kHz
  f_a = FreqNominalAntenna;
  radio_end_index = 0;
  median_mask_size = 99;
  NumOfAnalysedPeaks = 0;
  report_comment = QString("");
  t0 = 1;
  c_y0 = 0;
  c_w = FreqNominalAntenna;
  c_t0 = 1;
  c_th = 0;
  c_a0 = 1;

  ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                QCP::iSelectLegend | QCP::iSelectPlottables);

  ui->customPlot->xAxis->setRange(2e-5, 6e-5);
  ui->customPlot->yAxis->setRange(-2, 2);
  ui->customPlot->yAxis2->setRange(-0.025, 0.025);
  // ui->customPlot->yAxis->setScaleType(QCPAxis::stLogarithmic);
  ui->customPlot->axisRect()->setupFullAxesBox();

  ui->customPlot->plotLayout()->insertRow(0);
  QCPTextElement *title = new QCPTextElement(ui->customPlot, QObject::tr("Расчет параметров антенны"), QFont("sans", 17, QFont::Bold));
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
  connect(ui->action_estim_param, SIGNAL(triggered()), this, SLOT(estimate_params()));
  connect(ui->action_fit_curve, SIGNAL(triggered()), this, SLOT(open_fit_curve_toolbar()));

  // Parameters setting menus.
  connect(ui->action_set_f_nom, SIGNAL(triggered()), this, SLOT(SetFreqNominal()));
  connect(ui->action_set_Resonance_Freq, SIGNAL(triggered()), this, SLOT(SetFreqParRes()));
  connect(ui->action_set_R_meas, SIGNAL(triggered()), this, SLOT(SetRmeas()));
  connect(ui->action_set_n, SIGNAL(triggered()), this, SLOT(SetNPeaks()));
  connect(ui->action_comment, SIGNAL(triggered()), this, SLOT(SetComment()));

  QIDFreqNominal = new QInputDialog(this);
  QIDRmeas = new QInputDialog(this);
  QIDFreqParRes = new QInputDialog(this);
  QIDNPeaks = new QInputDialog(this); // Число анализируемых пиков.
  QIDComment = new QInputDialog(this); // Комментарий.

  QIDFreqNominal->setInputMode(QInputDialog::DoubleInput);
  QIDRmeas->setInputMode(QInputDialog::DoubleInput);
  QIDFreqParRes->setInputMode(QInputDialog::DoubleInput);
  QIDNPeaks->setInputMode(QInputDialog::IntInput);
  QIDComment->setInputMode(QInputDialog::TextInput);

  QIDFreqNominal->setComboBoxEditable(true);
  QIDRmeas->setComboBoxEditable(true);
  QIDFreqParRes->setComboBoxEditable(true);
  QIDNPeaks->setComboBoxEditable(true);
  // QIDComment->setComboBoxEditable(true);

  QIDFreqNominal->setDoubleMaximum(10000.0);
  QIDRmeas->setDoubleMaximum(1000.0);
  QIDFreqParRes->setDoubleMaximum(10000.0);
  QIDNPeaks->setIntMaximum(100);

  QIDFreqNominal->setDoubleMinimum(.001);
  QIDRmeas->setDoubleMinimum(0.001);
  QIDFreqParRes->setDoubleMinimum(.001);
  QIDNPeaks->setIntMinimum(1);

  QIDFreqNominal->setCancelButtonText(QObject::tr("Отмена"));
  QIDRmeas->setCancelButtonText(QObject::tr("Отмена"));
  QIDFreqParRes->setCancelButtonText(QObject::tr("Отмена"));
  QIDNPeaks->setCancelButtonText(QObject::tr("Отмена"));
  QIDComment->setCancelButtonText(QObject::tr("Отмена"));

  QIDFreqNominal->setOkButtonText(QObject::tr("Ввод"));
  QIDRmeas->setOkButtonText(QObject::tr("Ввод"));
  QIDFreqParRes->setOkButtonText(QObject::tr("Ввод"));
  QIDNPeaks->setOkButtonText(QObject::tr("Ввод"));
  QIDComment->setOkButtonText(QObject::tr("Ввод"));

  QIDFreqNominal->setLabelText(QObject::tr("Введите величину номинальной частоты резонанса антенны, кГц: "));
  QIDRmeas->setLabelText(QObject::tr("Укажите величину измерительного сопротивления, Ом: "));
  QIDFreqParRes->setLabelText(QObject::tr("Введите величину частоты параллельного резонанса антенны, кГц: "));
  QIDNPeaks->setLabelText(QObject::tr("Укажите число анализируемых полупериодов, не менее 3: "));
  QIDComment->setLabelText(QObject::tr("Комментарий: "));

  connect(QIDFreqNominal, SIGNAL(doubleValueChanged(double)), this, SLOT(changeFreqNominal(double)));
  connect(QIDFreqParRes, SIGNAL(doubleValueChanged(double)), this, SLOT(changeFreqParRes(double)));
  connect(QIDRmeas, SIGNAL(doubleValueChanged(double)), this, SLOT(changeRmeas(double)));
  connect(QIDNPeaks, SIGNAL(intValueChanged(int)), this, SLOT(changeNPeaks(int)));
  connect(QIDComment, SIGNAL(textValueChanged(QString)), this, SLOT(changeComment(QString)));

  create_fit_curve_toolbar();
}

void
MainWindow::create_parameters_setting_dialog() {
  QDParamDialog = new QDialog(this);
  QDParamDialog->setWindowTitle(QObject::tr("Значения параметров"));
  QDParamDialog->setMinimumSize(250, 100);

  // QVBoxLayout *layout = new QVBoxLayout;
  QGridLayout *gridLayout = new QGridLayout;

  QLabel *QLFreqNom = new QLabel(QObject::tr("Введите величину номинальной частоты резонанса антенны, кГц: "));
  QDoubleSpinBox *QSBFreqNominal = new QDoubleSpinBox();

  QLabel *QLRmeas = new QLabel(QObject::tr("Укажите величину измерительного сопротивления, Ом: "));
  QDoubleSpinBox *QSBRmeas = new QDoubleSpinBox();

  QLabel *QLFreqParaRes = new QLabel(QObject::tr("Введите величину частоты параллельного резонанса антенны, кГц: "));
  QDoubleSpinBox *QSBFreqParaRes = new QDoubleSpinBox();

  QLabel *QLComment = new QLabel(QObject::tr("Комментарий: "));
  QLineEdit *QLEComment = new QLineEdit();

  connect(QSBFreqNominal, SIGNAL(valueChanged(double)), this, SLOT(changeFreqNominal(double)));
  connect(QSBRmeas, SIGNAL(valueChanged(double)), this, SLOT(changeRmeas(double)));
  connect(QSBFreqParaRes, SIGNAL(valueChanged(double)), this, SLOT(changeFreqParRes(double)));
  connect(QLEComment, SIGNAL(textChanged(QString)), this, SLOT(changeComment(QString)));

  QSBFreqNominal->setMaximum(10000.0);
  QSBRmeas->setMaximum(1000.0);
  QSBFreqParaRes->setMaximum(10000.0);

  QSBFreqNominal->setMinimum(.001);
  QSBRmeas->setMinimum(0.001);
  QSBFreqParaRes->setMinimum(.001);

  QSBFreqNominal->setValue(FreqNominalAntenna);
  QSBRmeas->setValue(Rmeas);
  QSBFreqParaRes->setValue(FreqParRes);

  QPushButton *button_ok = new QPushButton("Ok");
  connect(button_ok, SIGNAL(pressed()), QDParamDialog, SLOT(hide()));
  connect(button_ok, SIGNAL(pressed()), this, SLOT(smooth()));

  gridLayout->addWidget(QLFreqNom, 0, 0);
  gridLayout->addWidget(QSBFreqNominal, 0, 1);
  gridLayout->addWidget(QLRmeas, 1, 0);
  gridLayout->addWidget(QSBRmeas, 1, 1);
  gridLayout->addWidget(QLFreqParaRes, 2, 0);
  gridLayout->addWidget(QSBFreqParaRes, 2, 1);
  gridLayout->addWidget(QLComment, 3, 0);
  gridLayout->addWidget(QLEComment, 3, 1);
  gridLayout->addWidget(button_ok, 4, 1);

  QDParamDialog->setLayout(gridLayout);
  QDParamDialog->show();
}

MainWindow::~MainWindow()
{
    delete ui;
}

/**
 * Reaction on menu button click.
 */
void MainWindow::open_csv_radio_dialog() {
	path_to_radio_csv = QFileDialog::getOpenFileName(this, QObject::tr("Укажите путь к радиоимпульсу."), "./../data/", QObject::tr("(*.csv)"));

	if(!path_to_radio_csv.isEmpty ()) {
		load_csv_radio();
	}
	else {
		QMessageBox::information(this, QObject::tr("SignalPlotter"),
														 QObject::tr("Файл не был открыт."));
	}

	open_csv_attenuation_dialog();
}

void MainWindow::open_csv_attenuation_dialog() {
	path_to_attenuation_csv = QFileDialog::getOpenFileName(this, QObject::tr("Укажите путь к затухающему сигналу."), path_to_radio_csv, QObject::tr("(*_CH2*);;(*.csv)"));

	if(!path_to_attenuation_csv.isEmpty()) {
		load_csv_attenuation();
	} else {
		QMessageBox::information(this, QObject::tr("SignalPlotter"),
														 QObject::tr("Файл не был открыт."));
	}

	ui->action_smooth->setEnabled(true);
  // Disable parameter calculation option.
  ui->action_estim_param->setEnabled(false);
  // Enable manual curve fitting.
  ui->action_fit_curve->setEnabled(true);

  // Dialog for all parameters.
  create_parameters_setting_dialog();
}

void MainWindow::save_report_dialog() {
  path_to_report =
      QFileDialog::getSaveFileName(this, QObject::tr("Укажите путь для сохранения."),
                                   QObject::tr("./../out/report.txt"), QObject::tr("(*.txt)"));

  if(!path_to_report.isEmpty ()) {
    save_report(path_to_report);
  }
  else {
    QMessageBox::information(this, QObject::tr("SignalPlotter"),
                             QObject::tr("Путь не был указан корректно."));
  }
}

/*
 * Wrapper under common .csv data loader.
 */
void MainWindow::load_csv_radio() {
	GraphParams g_params;

	samples_radio.clear();
	samples_radio_smoothed.clear();
	samples_radio = load_csv(path_to_radio_csv, &g_params);
	graph_radio = g_params;

	radio_end_index = find_radio_signal_termination(samples_radio);

	samples_radio.erase(samples_radio.begin() + radio_end_index, samples_radio.end());

	addGraph1(samples_radio, g_params, QString(QObject::tr("Радиовоздействие")), QColor(QString("green")));
}

void MainWindow::load_csv_attenuation() {
	GraphParams g_params;
	samples_attenuation.clear();
	samples_attenuation_smoothed.clear();
	samples_attenuation = load_csv(path_to_attenuation_csv, &g_params);
	graph_attenuation = g_params;

	addGraph2(samples_attenuation, g_params, QString(QObject::tr("Затухающий сигнал")), QColor(QString("blue")), true);
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

void
MainWindow::changeFreqNominal(double val) {
  FreqNominalAntenna = val;
  f_a = val;
}

void
MainWindow::changeFreqParRes(double val) {
  FreqParRes = val;
}

void
MainWindow::changeRmeas(double val) {
  Rmeas = val;
}

void
MainWindow::changeNPeaks(int val) {
  NumOfAnalysedPeaks = val;
}

void
MainWindow::changeComment(QString text) {
  report_comment = text;
}

void
MainWindow::SetFreqNominal() {
  QIDFreqNominal->setDoubleValue(FreqNominalAntenna);
  QIDFreqNominal->show();
}

void
MainWindow::SetFreqParRes() {
  QIDFreqParRes->setDoubleValue(FreqParRes);
  QIDFreqParRes->show();
}

void
MainWindow::SetRmeas() {
  QIDRmeas->setDoubleValue(Rmeas);
  QIDRmeas->show();
}

void
MainWindow::SetNPeaks() {
  QIDNPeaks->setIntValue(NumOfAnalysedPeaks);
  QIDNPeaks->show();
  ui->action_set_n_auto->setChecked(false);
}

void
MainWindow::change_cy0(double val) {
  c_y0 = val;
  updateGraph();
}

void
MainWindow::SetComment() {
  QIDComment->setTextValue(report_comment);
  QIDComment->show();
}

void
MainWindow::change_ca0(double val) {
  c_a0 = val;
  updateGraph();
}

void
MainWindow::change_cw(double val) {
  c_w = val;
  updateGraph();
}

void
MainWindow::change_ct0(double val) {
  c_t0 = val;
  updateGraph();
}

void
MainWindow::change_cth(double val) {
  c_th = val;
  updateGraph();
}

/**
 * Add graph to left hand side Y scale.
 * For radio signal.
 */
void MainWindow::addGraph1(Samples data, GraphParams const& g_params, QString const& message, QColor color, bool centralize) {
	// Determine size of current plot. (number of points in graph).
	int curSize;
	curSize = data.size();

	QVector<double> x(curSize);
	QVector<double> y(curSize);

	for (int i=0; i < curSize; i++)
	{
		x[i] = i * g_params.xScale + g_params.xOffset;
		y[i] = data[i] * g_params.yScale + g_params.yOffset;
	}

	ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis);
	ui->customPlot->graph()->setName(message);
	ui->customPlot->graph()->setData(x, y);
	ui->customPlot->graph()->setLineStyle(QCPGraph::lsLine);
	QPen graphPen;
	// graphPen.setColor(QColor(rand() % 245 + 10, rand() % 245 + 10, rand() % 245 + 10));
	graphPen.setColor(color);
	graphPen.setWidthF(1);
	ui->customPlot->graph()->setPen(graphPen);
	ui->customPlot->replot();
}

/**
 * Add graph to right hand side Y scale.
 * For damping oscillation.
 */
void MainWindow::addGraph2(Samples data, GraphParams const& g_params, QString const& message, QColor color, bool centralize) {
	// Determine size of current plot. (number of points in graph).
	int curSize;
	curSize = data.size();

	QVector<double> x(curSize);
	QVector<double> y(curSize);

	for (int i=0; i < curSize; i++)
	{
		x[i] = i * g_params.xScale + g_params.xOffset;
		y[i] = data[i] * g_params.yScale + g_params.yOffset;
	}

	double max = find_max(data) * g_params.yScale;
	double min = find_min(data) * g_params.yScale;

	if(centralize) {
    ui->customPlot->xAxis->setRange(g_params.xOffset - x.size() * 0.1 * g_params.xScale, 1.1 * x.size() * g_params.xScale + g_params.xOffset);
    // ui->customPlot->yAxis2->setRange(g_params.yOffset - 10 * g_params.yScale, (y.size() + 10) * g_params.yScale + g_params.yOffset);
    ui->customPlot->yAxis->setRange(1.5 * min, 1.5 * max);
	}

	// Attenuation graph has different y scale.
	ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis2);
	ui->customPlot->graph()->setName(message);
	ui->customPlot->graph()->setData(x, y);
	ui->customPlot->graph()->setLineStyle(QCPGraph::lsLine);
	QPen graphPen;
	// graphPen.setColor(QColor(rand() % 245 + 10, rand() % 245 + 10, rand() % 245 + 10));
	graphPen.setColor(color);
	graphPen.setWidthF(1);
	ui->customPlot->graph()->setPen(graphPen);
	ui->customPlot->replot();
}

/*
 * For exponential asimptots.
 */
void MainWindow::addGraph3(Samples data, GraphParams const& g_params, QString const& message, QColor color, bool centralize) {
  // Determine size of current plot. (number of points in graph).
  int curSize;
  curSize = data.size();

  QVector<double> x(curSize);
  QVector<double> y(curSize);

  for (int i=0; i < curSize; i++)
  {
    x[i] = i * g_params.xScale + g_params.xOffset;
    y[i] = data[i] * g_params.yScale + g_params.yOffset;
  }

  // Attenuation graph has different y scale.
  ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis2);
  ui->customPlot->graph()->setName(message);
  ui->customPlot->graph()->setData(x, y);
  ui->customPlot->graph()->setLineStyle(QCPGraph::lsLine);
  QPen graphPen;
  // graphPen.setColor(QColor(rand() % 245 + 10, rand() % 245 + 10, rand() % 245 + 10));
  graphPen.setColor(color);
  graphPen.setWidthF(1);
  ui->customPlot->graph()->setPen(graphPen);
  ui->customPlot->replot();
}

void MainWindow::addGraph4(Samples data, GraphParams const& g_params, QString const& message, QColor color, bool centralize) {
  // Determine size of current plot. (number of points in graph).
  int curSize;
  curSize = data.size();

  QVector<double> x(curSize);
  QVector<double> y(curSize);

  for (int i=0; i < curSize; i++)
  {
    x[i] = i * g_params.xScale + g_params.xOffset;
    y[i] = data[i] * g_params.yScale + g_params.yOffset;
  }

  // Attenuation graph has different y scale.
  ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis2);
  ui->customPlot->graph()->setName(message);
  ui->customPlot->graph()->setData(x, y);
  ui->customPlot->graph()->setLineStyle(QCPGraph::lsLine);
  QPen graphPen;
  // graphPen.setColor(QColor(rand() % 245 + 10, rand() % 245 + 10, rand() % 245 + 10));
  graphPen.setColor(color);
  graphPen.setWidthF(3);
  ui->customPlot->graph()->setPen(graphPen);
  ui->customPlot->replot();
}

void MainWindow::updateGraph() {
  // Remove all graphs.
  removeAllGraphs();

  // Add graphs.
  if(!samples_radio.empty())
    addGraph1(samples_radio, graph_radio, QObject::tr("Радиовоздействие"), QColor(QString("green")));
  if(!samples_attenuation.empty())
    addGraph2(samples_attenuation, graph_attenuation, QObject::tr("Затухающий сигнал"), QColor(QString("blue")));
  if(!samples_radio_smoothed.empty())
    addGraph1(samples_radio_smoothed, graph_radio, QObject::tr("Радиовоздействие, ФНЧ"), QColor(QString("yellow")));
  if(!samples_attenuation_smoothed.empty())
    addGraph2(samples_attenuation_smoothed, graph_attenuation, QObject::tr("Затухающий сигнал, ФНЧ"), QColor(QString("red")));

  if(!fitting_curve.empty() && ui->action_fit_curve->isChecked()) {
    recalculate_param_curve();
    addGraph4(fitting_curve, graph_fitting_curve, QObject::tr("Параметрическая кривая"), QColor(QString("orange")));
  }
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
    menu->addAction(QObject::tr("Показать выбранные кривые"), this, SLOT(showSelectedGraph()));
    // menu->addAction("Show selected graph", this, SLOT(showSelectedGraph()));
  if (ui->customPlot->selectedGraphs().size() > 0)
    menu->addAction(QObject::tr("Скрыть выбранные кривые"), this, SLOT(hideSelectedGraph()));
    // menu->addAction("Hide selected graph", this, SLOT(hideSelectedGraph()));
  if (ui->customPlot->selectedGraphs().size() > 0)
    menu->addAction(QObject::tr("Удалить выбранные кривые"), this, SLOT(removeSelectedGraph()));
    // menu->addAction("Remove selected graph", this, SLOT(removeSelectedGraph()));
  if (ui->customPlot->graphCount() > 0)
    menu->addAction(QObject::tr("Удалить все кривые"), this, SLOT(removeAllGraphs()));
    // menu->addAction("Remove all graphs", this, SLOT(removeAllGraphs()));

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

	if (!samples_radio.empty()) {
	  // Smooth radio signal with low-pass filter.
		samples_radio_smoothed = lp_ampl(samples_radio, graph_radio.xScale, freq_factor_to_pass * 5 * FreqNominalAntenna * 1000);
		updateGraph();
	}

	if(!samples_attenuation.empty()) {
	  // Smooth oscillation signal with low-pass filter.
		samples_attenuation_smoothed = lp_ampl(samples_attenuation, graph_attenuation.xScale, freq_factor_to_pass * FreqNominalAntenna * 1000);

		// Find symmetry of signal relative to Ox and move. It's not precise but mostly it helps.
		centrate_signal_ox(samples_attenuation_smoothed, 0, samples_attenuation_smoothed.size());

		updateGraph();
	}
	// Enable parameter calculation option.
	ui->action_estim_param->setEnabled(true);
}

int
MainWindow::estimate_params() {
  int ret = 0;
  if(ui->action_fit_curve->isChecked()) {
    printf("estimate_contour_params_hand sig - start\n");
    ret = estimate_contour_params_hand();
  }
  else {
    printf("estimate_contour_params sig - start\n");
    ret = estimate_contour_params();
  }

  return ret;
}

int
MainWindow::estimate_contour_params() {
  printf("estimate_contour_params\n");
	double w0, w, b;

	Samples exp_curve, exp_curve_neg;

	// Estimate attenuation signal parameters: exp low, q-factor, frequency.
	if(signal_analyzer(&t0, &b, &Q, &f_a) != 0) {
	  return -1;
	}

	// Plot exponential asymptotes.
	updateGraph();
	for(unsigned int i = radio_end_index; i < samples_attenuation_smoothed.size(); ++i) {
		exp_curve.push_back(exp(t0 * i + b));
		exp_curve_neg.push_back(-exp(t0 * i + b));
	}
	GraphParams graph_exp = graph_attenuation;
	graph_exp.xOffset = graph_attenuation.xOffset + radio_end_index * graph_attenuation.xScale;
	addGraph3(exp_curve, graph_exp, QString(QObject::tr("Асимптота верхняя")), QColor(QString("gray")));
	addGraph3(exp_curve_neg, graph_exp, QString(QObject::tr("Асимптота нижняя")), QColor(QString("gray")));

	// Estimate Umax, Imax.
	U_max = find_max(samples_radio) - find_min(samples_radio);
	I_max = 2 * exp_curve.front() / Rmeas;
	Ra = U_max / I_max;
  w = 2 * M_PI * f_a;
	double delta = -t0 / graph_attenuation.xScale;
	printf("t0 = %f\n", t0);
	printf("Amax = %f\n", find_max(Samples(samples_attenuation_smoothed.begin() + radio_end_index, samples_attenuation_smoothed.end())));
	printf("Amax_exp = %f\n", exp_curve.front());
  La = Ra / (2 * delta);
  w0 = sqrt(w * w + delta * delta);
  Ca = 1.0 / (La * w0 * w0);
  F0 = w0 / (2 * M_PI);
  C0 = Ca / ((FreqParRes * 1000 / F0) * (FreqParRes * 1000 / F0) - 1);

  // Show message window.
  QString scout;
  scout = QObject::tr("Заданные параметры\n");
  scout += QString(report_comment + "\n\n");
  scout += (QObject::tr("Rmeas:             ") + QString::number(Rmeas) + QObject::tr(", Ом;") + "\n");
  scout += (QObject::tr("Fnom:              ") + QString::number(FreqNominalAntenna) + QObject::tr(", кГц;") + "\n");
  scout += (QObject::tr("F, пар. рез.:     ") + QString::number(FreqParRes) + QObject::tr(", кГц;") + "\n\n");
  scout += QObject::tr("Измеренные вспомогательные параметры(по экспоненте)\n");
  scout += (QObject::tr("Umax, размах:     ") + QString::number(U_max * 1000) + QObject::tr(", мВ;") + "\n");
  scout += (QObject::tr("Imax, размах:       ") + QString::number(I_max * 1000) + QObject::tr(", мA;") + "\n");
  scout += (QObject::tr("Добротность:       ") + QString::number(Q) + ";\n\n");
  scout += QObject::tr("Параметры контура\n");
  scout += ("Ra:     " + QString::number(Ra) + QObject::tr(", Oм;") + "\n");
  scout += ("Ca:     " + QString::number(Ca * 1e12) + QObject::tr(", пФ;") + "\n");
  scout += ("La:     " + QString::number(La * 1e6) + QObject::tr(", мкГн;") + "\n");
  scout += ("C0:     " + QString::number(C0 * 1e12) + QObject::tr(", пФ;") + "\n");
  scout += (QObject::tr("F0, частота колебательного контура:     ") + QString::number(F0 / 1000) + QObject::tr(", кГц;") + "\n");
  scout += (QObject::tr("F, частота свободных колебаний:           ") + QString::number(w / (2000 * M_PI)) + QObject::tr(", кГц") + "\n");

  QMessageBox::information(this, QObject::tr("Протокол измерения параметров антенны"), scout);

  // Save report into folder with data.
  QString report_path;
  QString report_name;

  QDateTime cur_date = QDateTime::currentDateTime();
  report_path = path_from_fullname(path_to_attenuation_csv);
  report_name = QString("report_%1_%2_%3_%4").arg(cur_date.date().month())
                                             .arg(cur_date.date().year())
                                             .arg(cur_date.date().day())
                                             .arg(cur_date.time().secsTo(QTime(0, 0)));

  save_report(QString(report_path + report_name + ".txt"));

  return 0;
}


int
MainWindow::estimate_contour_params_hand() {
  double w0;

  Samples exp_curve, exp_curve_neg;

  // Plot exponential asymptotes.
  for(unsigned int i = radio_end_index; i < samples_attenuation_smoothed.size(); ++i) {
    double x = (i - radio_end_index);
    exp_curve.push_back(c_y0 + c_a0 * exp(-x / c_t0));
    exp_curve_neg.push_back(c_y0 - c_a0 * exp(-x / c_t0));
  }
  GraphParams graph_exp = graph_attenuation;
  graph_exp.xOffset = graph_attenuation.xOffset + radio_end_index * graph_attenuation.xScale;
  addGraph3(exp_curve, graph_exp, QString(QObject::tr("Асимптота верхняя")), QColor(QString("gray")));
  addGraph3(exp_curve_neg, graph_exp, QString(QObject::tr("Асимптота нижняя")), QColor(QString("gray")));

  // Estimate Umax, Imax.
  U_max = find_max(samples_radio) - find_min(samples_radio);
  printf("Rmeas = %f\n", Rmeas);
  I_max = 2 * exp_curve.front() / Rmeas;
  if(fabs(I_max) > eps) {
    Ra = U_max / I_max;
  }
  else {
    printf("Error: estimate_contour_params_hand: division by zero I_max = %f\n", I_max);
    return -1;
  }

  double delta = 0;
  if(fabs(graph_attenuation.xScale) > eps) {
    delta = 1 /(c_t0 * graph_attenuation.xScale);
  }
  else {
    printf("Error: estimate_contour_params_hand: division by zero, graph_attenuation.xScale = %.51f\n", graph_attenuation.xScale);
    return -1;
  }

  if(fabs(delta) > eps) {
    double d = 2 * M_PI / (c_t0 * c_w * graph_attenuation.xScale);
    Q = M_PI / d;
    printf("t0 = %f\n", t0);
    printf("Amax = %f\n", find_max(Samples(samples_attenuation_smoothed.begin() + radio_end_index, samples_attenuation_smoothed.end())));
    printf("Amax_exp = %f\n", exp_curve.front());
    La = Ra / (2 * delta);
  }
  else {
    printf("Error: estimate_contour_params_hand: division by zero, delta = %f\n", delta);
    return -1;
  }

  w0 = sqrt(c_w * c_w + delta * delta);
  if (fabs(La) > eps || w0 > eps) {
    Ca = 1.0 / (La * w0 * w0);
    F0 = w0 / (2 * M_PI);
    C0 = Ca / ((FreqParRes * 1000 / F0) * (FreqParRes * 1000 / F0) - 1);
  }
  else {
    printf("Error: estimate_contour_params_hand: division by zero, w0 = %f || La = %f \n", w0, La);
    return -1;
  }

  // Show message window.
  QString scout;
  scout = QObject::tr("Заданные параметры\n");
  scout += QString(report_comment + "\n\n");
  scout += (QObject::tr("Rmeas:             ") + QString::number(Rmeas) + QObject::tr(", Ом;") + "\n");
  scout += (QObject::tr("Fnom:              ") + QString::number(FreqNominalAntenna) + QObject::tr(", кГц;") + "\n");
  scout += (QObject::tr("F, пар. рез.:     ") + QString::number(FreqParRes) + QObject::tr(", кГц;") + "\n\n");
  scout += QObject::tr("Измеренные вспомогательные параметры(по экспоненте)\n");
  scout += (QObject::tr("Umax, размах:     ") + QString::number(U_max * 1000) + QObject::tr(", мВ;") + "\n");
  scout += (QObject::tr("Imax, размах:       ") + QString::number(I_max * 1000) + QObject::tr(", мA;") + "\n");
  scout += (QObject::tr("Добротность:       ") + QString::number(Q) + ";\n\n");
  scout += QObject::tr("Параметры контура\n");
  scout += ("Ra:     " + QString::number(Ra) + QObject::tr(", Oм;") + "\n");
  scout += ("Ca:     " + QString::number(Ca * 1e12) + QObject::tr(", пФ;") + "\n");
  scout += ("La:     " + QString::number(La * 1e6) + QObject::tr(", мкГн;") + "\n");
  scout += ("C0:     " + QString::number(C0 * 1e12) + QObject::tr(", пФ;") + "\n");
  scout += (QObject::tr("F0, частота колебательного контура:     ") + QString::number(F0 / 1000) + QObject::tr(", кГц;") + "\n");
  scout += (QObject::tr("F, частота свободных колебаний:           ") + QString::number(c_w / (2000 * M_PI)) + QObject::tr(", кГц") + "\n");

  QMessageBox::information(this, QObject::tr("Протокол измерения параметров антенны"), scout);

  // Save report into folder with data.
  QString report_path;
  QString report_name;

  QDateTime cur_date = QDateTime::currentDateTime();
  report_path = path_from_fullname(path_to_attenuation_csv);
  report_name = QString("report_%1_%2_%3_%4").arg(cur_date.date().month())
                                             .arg(cur_date.date().year())
                                             .arg(cur_date.date().day())
                                             .arg(cur_date.time().secsTo(QTime(0, 0)));

  save_report(QString(report_path + report_name + ".txt"));

  return 0;
}

/**
 * Interface function for signal analysis.
 */
int
MainWindow::signal_analyzer(double *a, double *b, double *q_factor, double *freq) {
	unsigned int start = radio_end_index;
	unsigned int finish = samples_attenuation_smoothed.size() - 1;

	std::vector<unsigned int> zero_points = sign_changes(samples_attenuation_smoothed, start, finish);

	if(zero_points.size() < 4) {
	  QMessageBox::information(this, QObject::tr("Ошибка"), QObject::tr("Невозможно произвести рассчеты из-за малого числа пересечений с 0x."));
	  return -1;
	}

	Peaks all_peaks = find_all_peaks(samples_attenuation_smoothed, zero_points);

	Peaks real_peaks;
	if(NumOfAnalysedPeaks > 2 && !ui->action_set_n_auto->isChecked()) {
	  real_peaks = Peaks(all_peaks.begin(), all_peaks.begin() + NumOfAnalysedPeaks);
	  zero_points = std::vector<unsigned int>(zero_points.begin(), zero_points.begin() + NumOfAnalysedPeaks + 1);
	}
	else {
	  real_peaks = find_real_peaks(zero_points, samples_attenuation_smoothed, all_peaks, 0.3);
	}

	if(real_peaks.size() < 3) {
	  printf("Error: MainWindow::signal_analyzer: real_peaks.size() < 3");
	  QMessageBox::information(this, QObject::tr("Ошибка"), QObject::tr("Невозможно произвести расчеты из-за малого числа пересечений с Ox."));
	  return -1;
	}

	estimate_quality_ls(a, b, samples_attenuation_smoothed, real_peaks, radio_end_index);

	if(!verify_half_periods(zero_points)) {
	  fit_in_exp_bound(samples_attenuation_smoothed, samples_attenuation_smoothed, real_peaks, *a, *b, radio_end_index);
	  updateGraph();
	}

	theta = (zero_points.front() - radio_end_index) * graph_attenuation.xScale;

	*freq = estimate_frequency(real_peaks, graph_attenuation.xOffset, graph_attenuation.xScale);

	double d = - *a / (*freq * graph_attenuation.xScale);

	*q_factor = M_PI / d;

	return 0;
}

void
MainWindow::create_fit_curve_toolbar() {
  // y = c_y0 + c_a0 * exp(c_t0 * x) * sin(c_w * x + c_th)
  QTBCurveFit = addToolBar(tr("Curve Fit"));
  QTBCurveFit->setAllowedAreas(Qt::RightToolBarArea);
  QWidget *QWCurveFitWidget = new QWidget(this);

  QGridLayout *gridLayout = new QGridLayout;
  QVBoxLayout *vertical_layout = new QVBoxLayout;

  // QWCurveFitWidget->setLayout(gridLayout);
  QWCurveFitWidget->setLayout(vertical_layout);

  QLabel *QLBL_formulae = new QLabel("y = y0 + A0 * exp(-t / t0) * sin(w * (t - theta))");
  QLabel *QLBL_y0 = new QLabel(QObject::tr("y0, В:"));
  QDSB_cy0 = new QDoubleSpinBox();
  QLabel *QLBL_a0 = new QLabel(QObject::tr("A0, В:"));
  QDSB_ca0 = new QDoubleSpinBox();
  QLabel *QLBL_w = new QLabel(QObject::tr("w, рад/с:"));
  QDSB_cw = new QDoubleSpinBox();
  QLabel *QLBL_t0 = new QLabel(QObject::tr("t0, с:"));
  QDSB_ct0 = new QDoubleSpinBox();
  QLabel *QLBL_th = new QLabel(QObject::tr("theta, с:"));
  QDSB_cth = new QDoubleSpinBox();

  QDSB_cy0->setMaximum(10000.0);
  QDSB_ca0->setMaximum(10000.0);
  QDSB_ct0->setMaximum(100000000.0);
  QDSB_cw->setMaximum(1000000000.0);
  QDSB_cth->setMaximum(100000000.0);

  QDSB_cy0->setMinimum(-10000.0);
  QDSB_ca0->setMinimum(0.0);
  QDSB_ct0->setMinimum(-100000000.0);
  QDSB_cw->setMinimum(.0001);
  QDSB_cth->setMinimum(-100000000.0);

  QDSB_cy0->setSingleStep(0.0001);
  QDSB_ca0->setSingleStep(0.001);
  QDSB_ct0->setSingleStep(0.0000025);
  QDSB_cw->setSingleStep(0.1);
  QDSB_cth->setSingleStep(0.0000001);

  QDSB_cy0->setDecimals(18);
  QDSB_ca0->setDecimals(5);
  QDSB_ct0->setDecimals(8);
  QDSB_cw->setDecimals(2);
  QDSB_cth->setDecimals(9);

  QDSB_cy0->setValue(c_y0);
  QDSB_ca0->setValue(c_a0);
  QDSB_ct0->setValue(c_t0);
  QDSB_cw->setValue(c_w);
  QDSB_cth->setValue(c_th);

  connect(QDSB_cy0, SIGNAL(valueChanged(double)), this, SLOT(change_cy0(double)));
  connect(QDSB_ca0, SIGNAL(valueChanged(double)), this, SLOT(change_ca0(double)));
  connect(QDSB_ct0, SIGNAL(valueChanged(double)), this, SLOT(change_ct0(double)));
  connect(QDSB_cw, SIGNAL(valueChanged(double)), this, SLOT(change_cw(double)));
  connect(QDSB_cth, SIGNAL(valueChanged(double)), this, SLOT(change_cth(double)));

  gridLayout->addWidget(QLBL_y0, 0, 0);
  gridLayout->addWidget(QDSB_cy0, 0, 1);
  gridLayout->addWidget(QLBL_a0, 1, 0);
  gridLayout->addWidget(QDSB_ca0, 1, 1);
  gridLayout->addWidget(QLBL_w, 2, 0);
  gridLayout->addWidget(QDSB_cw, 2, 1);
  gridLayout->addWidget(QLBL_t0, 3, 0);
  gridLayout->addWidget(QDSB_ct0, 3, 1);
  gridLayout->addWidget(QLBL_th, 4, 0);
  gridLayout->addWidget(QDSB_cth, 4, 1);

  vertical_layout->addWidget(QLBL_formulae);
  vertical_layout->addLayout(gridLayout);

  QTBCurveFit->addWidget(QWCurveFitWidget);
  QTBCurveFit->hide();
}

void
MainWindow::open_fit_curve_toolbar() {
  if(ui->action_fit_curve->isChecked()) {
    if (I_max != 0) {
      c_a0 = I_max * Rmeas / 2;
      // c_a0 = find_max(Samples(samples_attenuation_smoothed.begin() + radio_end_index, samples_attenuation_smoothed.end())); // debug
    } else {
      c_a0 = find_max(Samples(samples_attenuation_smoothed.begin() + radio_end_index, samples_attenuation_smoothed.end()));
    }
    c_t0 = - 1 / t0;
    printf("c_t0 = %f\n", c_t0);
    // c_y0 = c_a0 - fabs(find_min(samples_attenuation_smoothed));
    // c_y0 = find_mean(samples_attenuation_smoothed);
    c_y0 = fabs(find_min(samples_attenuation_smoothed)) - fabs(find_max(samples_attenuation_smoothed));
    c_w = f_a * (2 * M_PI);
    c_th = theta;

    QDSB_cy0->setValue(c_y0);
    QDSB_ca0->setValue(c_a0);
    QDSB_ct0->setValue(c_t0);
    QDSB_cw->setValue(c_w);
    QDSB_cth->setValue(c_th);
    QTBCurveFit->show();
    recalculate_param_curve();
    updateGraph();
  }
  else {
    QTBCurveFit->hide();
  }
}

void
MainWindow::recalculate_param_curve() {
  std::vector<double> x(fit_curve_size);

  fitting_curve.clear();

  // Decimation factor.
  int dec_factor = int(double(samples_attenuation.size()) / fit_curve_size);

  // Form x and y vector.
  for(unsigned int i = radio_end_index; i < samples_attenuation.size(); ++i) {
    if(i % dec_factor == 0) {
      x.push_back(i * graph_attenuation.xScale + graph_attenuation.xOffset); // + radio_end_index * graph_attenuation.xScale);
      fitting_curve.push_back( y(x.back()) );
    }
  }

  graph_fitting_curve = graph_attenuation;
  graph_fitting_curve.xScale = graph_attenuation.xScale * dec_factor;
  graph_fitting_curve.xOffset = graph_attenuation.xOffset + radio_end_index * graph_attenuation.xScale;
}

void
MainWindow::save_report(QString const& filepath) {

  QFile file(filepath.toStdString().c_str());

  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
            return;

  QTextStream out(&file);
  // out.setCodec("Windows-1251");
  out.setCodec("UTF-8");

  // Extract folder name.
  QStringList qsl_full_path = filepath.split("/");
  qsl_full_path.pop_back();
  QString folder_name = qsl_full_path.back();

  // Print parameters to report;
  QString scout;
  scout = QObject::tr("Протокол измерения параметров антенны\n\n");
  scout += QString(report_comment + "\n\n");
  scout += QString(QObject::tr("Папка: ") + folder_name + ".\n\n");
  scout += QObject::tr("Заданные параметры:\n");
  scout += (QObject::tr("Rmeas:             ") + QString::number(Rmeas) + QObject::tr(", Ом;") + "\n");
  scout += (QObject::tr("Fnom:              ") + QString::number(FreqNominalAntenna) + QObject::tr(", кГц;") + "\n");
  scout += (QObject::tr("F, пар. рез.:      ") + QString::number(FreqParRes) + QObject::tr(", кГц;") + "\n\n");
  scout += QObject::tr("Измеренные вспомогательные параметры(по экспоненте):\n");
  scout += (QObject::tr("Umax, размах:      ") + QString::number(U_max * 1000) + QObject::tr(", мВ;") + "\n");
  scout += (QObject::tr("Imax, размах:      ") + QString::number(I_max * 1000) + QObject::tr(", мA;") + "\n");
  scout += (QObject::tr("Добротность:       ") + QString::number(Q) + ";\n\n");
  scout += QObject::tr("Параметры контура:\n");
  scout += ("Ra:     " + QString::number(Ra) + QObject::tr(", Oм;") + "\n");
  scout += ("Ca:     " + QString::number(Ca * 1e12) + QObject::tr(", пФ;") + "\n");
  scout += ("La:     " + QString::number(La * 1e6) + QObject::tr(", мкГн;") + "\n");
  scout += ("C0:     " + QString::number(C0 * 1e12) + QObject::tr(", пФ;") + "\n");
  scout += (QObject::tr("F0, частота колебательного контура:     ") + QString::number(F0 / 1000) + QObject::tr(", кГц;") + "\n");
  scout += (QObject::tr("F, частота свободных колебаний:         ") + QString::number(f_a / 1000) + QObject::tr(", кГц") + "\n");

  if(ui->action_fit_curve->isChecked()) {
    scout += QObject::tr("\nПараметры, рассчитанные по параметрической кривой: \n");

    scout += QObject::tr("\nПараметры кривой (отладочная информация): \n");
    scout += (QString("c_y0 = %1\n, c_w = %2\n, c_t0 = %3\n, c_th = %4\n, c_a0 = %5").arg(c_y0, 0, 'E', 21)
                                                                                     .arg(c_w, 0, 'E', 21)
                                                                                     .arg(c_t0, 0, 'E', 21)
                                                                                     .arg(c_th, 0, 'E', 21)
                                                                                     .arg(c_a0, 0, 'E', 21) + "\n");

    scout += QObject::tr("Соотвтетствующие вычисленные параметры: \n");
    scout += (QString("y0 = %1, f_a = %2, t0 = %3, c_a0 = %4").arg(find_mean(samples_attenuation_smoothed), 0, 'E', 21)
                                                              .arg(f_a, 0, 'E', 21)
                                                              .arg(t0, 0, 'E', 21)
                                                              .arg(find_max(samples_attenuation_smoothed), 0, 'E', 21) + "\n");
  }

  out << scout;
}

bool
MainWindow::verify_half_periods(std::vector<unsigned int> const& zero_points) {
  bool ret = true;
	float max_dev, mean_dev;

	half_periods_verificator(zero_points, &max_dev, &mean_dev);

	std::vector<QPoint> xy_points;

	for(unsigned int i = 0; i < zero_points.size(); ++i) {
		xy_points.push_back(QPoint(zero_points[i], 0));
	}
	plot_points(xy_points, graph_attenuation);

	printf("max_dev = %f, mean_dev = %f\n", max_dev, mean_dev);

	if(max_dev > 0.03) {
		QMessageBox::information(this, QObject::tr("SignalPlotter"),
														 QObject::tr("Предупреждение: максимальное отклонение в измерении полупериодов превысило 3% и равно %1 %.").arg(max_dev * 100));
		ret = false;
	}

	return ret;
}

void
MainWindow::plot_points(std::vector<QPoint> const& xy_points, GraphParams const& g_params) {
	QVector<double> x, y;
	for(unsigned int i = 0; i < xy_points.size(); ++i){
		x.push_back(xy_points[i].x() * g_params.xScale + g_params.xOffset);
		y.push_back(xy_points[i].y() * g_params.yScale + g_params.yOffset);
	}

	ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis);
	ui->customPlot->graph()->setName(QString("Points"));
	ui->customPlot->graph()->setData(x, y);
	ui->customPlot->graph()->setLineStyle(QCPGraph::lsNone);
	ui->customPlot->graph()->setScatterStyle(QCPScatterStyle::ssCircle);
	ui->customPlot->graph()->setPen(QPen(QBrush(Qt::red), 2));
	ui->customPlot->replot();
}

/*
 * Deprecated.
 */
bool
MainWindow::verify_half_periods(Intervals const& zero_intervals) {
  bool ret = true;
	float max_dev, mean_dev;

	half_periods_verificator(zero_intervals, &max_dev, &mean_dev);

	std::vector<QPoint> xy_points;

	for(unsigned int i = 0; i < zero_intervals.size(); ++i) {
		xy_points.push_back(QPoint(zero_intervals[i].first, 0));
		xy_points.push_back(QPoint(zero_intervals[i].second, 0));
	}
	plot_points(xy_points, graph_attenuation);

	printf("max_dev = %f, mean_dev = %f\n", max_dev, mean_dev);

	if(max_dev > 0.03) {
		QMessageBox::information(this, QObject::tr("SignalPlotter"),
														 QObject::tr("Предупреждение: отклонение в измерении полупериодов превысило 3%!"));
		ret = false;
	}

  return ret;
}

double
MainWindow::y(double x) {
  double i = (x - graph_attenuation.xOffset - radio_end_index * graph_attenuation.xScale) / graph_attenuation.xScale;

  return c_y0 + c_a0 * exp(-i / c_t0) * sin(c_w * (x - c_th));
}

#endif
