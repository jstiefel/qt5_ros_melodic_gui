#include <QtWidgets> //includes all of Qt's GUI classes
#include <QMessageBox> //for information, about etc.
#include <iostream>
#include "nav_ui/mainwindow.hpp"

namespace nav_ui {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  qnode.init(); //initializes ROS node and starts QThread with spin
  ui.setupUi(this); //initializing ui, calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  setWindowTitle(QString("Kinematic Control Interface"));
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tabWidget->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  //State:
  QObject::connect(&qnode, SIGNAL(stateUpdated()), this, SLOT(updateStateView()));

  // Logging:
  ui.view_logging->setModel(qnode.loggingModel()); //set a model
  //if QSignal loggingUpdated, execute updateLoggingView
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

}

MainWindow::~MainWindow() {}

// slot implementation
void MainWindow::on_ee_homing_clicked()
{
    qnode.homing();
}

void MainWindow::on_q_homing_clicked()
{
    qnode.homing();
}

void MainWindow::on_ee_start_clicked()
{
    qnode.ee_target_pos((float)ui.ee_y->value(), (float)ui.ee_x->value(), (float)ui.ee_z->value(), (float)ui.ee_phi->value(), (float)ui.ee_theta->value());
}

void MainWindow::on_q_start_clicked()
{
    qnode.joint_target_pos((float)ui.q_y->value(), (float)ui.q_x->value(), (float)ui.q_alpha->value(), (float)ui.q_beta->value(), (float)ui.q_d->value());
}

void MainWindow::on_q_injection_clicked()
{
    qnode.make_injection();
}

void MainWindow::on_ee_injection_clicked()
{
    qnode.make_injection();
}


// menu implementation

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About"),tr("<h2>Kinematic Control Interface</h2><p>Author: Julian Stiefel</p>"));
}

void MainWindow::on_actionQuit_triggered()
{
  QApplication::quit();
}

void MainWindow::on_actionManual_triggered()
{
    QMessageBox::information(this, tr("Manual"), tr("<h2>Manual</h2><p>Please take a look at the instructions.</p>"));
}

void MainWindow::on_copy_to_control_clicked()
{
    ui.q_x->setValue((double)qnode.x_value);
    ui.q_y->setValue((double)qnode.y_value);
    ui.q_alpha->setValue((double)qnode.alpha_value);
    ui.q_beta->setValue((double)qnode.beta_value);
    ui.q_d->setValue((double)qnode.d_value);
    ui.ee_x->setValue((double)qnode.x_ee_value);
    ui.ee_y->setValue((double)qnode.y_ee_value);
    ui.ee_z->setValue((double)qnode.z_ee_value);
}

//Logging:

/**
 * This function is signaled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void MainWindow::updateStateView() {
  ui.q_actual_x->display(qnode.x_value);
  ui.q_actual_y->display(qnode.y_value);
  ui.q_actual_alpha->display(qnode.alpha_value);
  ui.q_actual_beta->display(qnode.beta_value);
  ui.q_actual_d->display(qnode.d_value);
  ui.ee_actual_x->display(qnode.x_ee_value);
  ui.ee_actual_y->display(qnode.y_ee_value);
  ui.ee_actual_z->display(qnode.z_ee_value);
  ui.homing_completed->setChecked(qnode.homing_completed_status);
  ui.position_reached->setChecked(qnode.last_position_reached_status);
}

// implementation configuration

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

}  // namespace nav_ui




