/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "main_window.hpp"

#include "cglobal.hpp"
#include <QTime>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace stellax_gui_node {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    ReadSettings();
    setWindowIcon(QIcon(":/images/stellax_icon.jpg"));
    setWindowTitle("stellax_gui");
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    ui.checkbox_use_environment->setEnabled(false);
    ui.checkbox_remember_settings->setEnabled(false);

    ui.btn_start_roscore->setStyleSheet("background-color: rgb(255,255,0);");

    ui.btn_up->setStyleSheet("background-color: rgb(127, 255, 0);");
    ui.btn_down->setStyleSheet("background-color: rgb(127, 255, 0);");
    ui.btn_left->setStyleSheet("background-color: rgb(127, 255, 0);");
    ui.btn_right->setStyleSheet("background-color: rgb(127, 255, 0);");
    ui.btn_stop->setStyleSheet("background-color: rgb(0, 255, 255);");

    ui.btn_keyboard_teleop->setStyleSheet("background-color: rgb(202,235,216);");
    ui.btn_make_a_map->setStyleSheet("background-color: rgb(202,235,216);");
    ui.btn_save_a_map->setStyleSheet("background-color: rgb(202,235,216);");
    ui.btn_navigation->setStyleSheet("background-color: rgb(202,235,216);");
    ui.btn_person_follower->setStyleSheet("background-color: rgb(202,235,216);");

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    // close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "stellax_gui_node");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "stellax_gui_node");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace stellax_gui_node


void stellax_gui_node::MainWindow::on_btn_start_roscore_clicked()
{
    static bool flag_start_roscore = true;
        if (flag_start_roscore) {
             ui.btn_start_roscore->setStyleSheet("background-color: rgb(255,0,0);");
             ui.btn_start_roscore->setText("close roscore");
            system("gnome-terminal -x bash -c 'source /opt/ros/indigo/setup.bash;roscore; limited:=true'&");
            flag_start_roscore = false;
        } else {
            ui.btn_start_roscore->setStyleSheet("background-color: rgb(255,255,0);");
            ui.btn_start_roscore->setText("start roscore");
            system("killall -g roscore");
            flag_start_roscore = true;
        }
}

void stellax_gui_node::MainWindow::on_btn_up_pressed()
{
    CGlobal::key_value = 1;
}

void stellax_gui_node::MainWindow::on_btn_up_released()
{
    CGlobal::key_value = -1;
}

void stellax_gui_node::MainWindow::on_btn_down_pressed()
{
    CGlobal::key_value = 2;
}

void stellax_gui_node::MainWindow::on_btn_down_released()
{
    CGlobal::key_value = -1;
}

void stellax_gui_node::MainWindow::on_btn_left_pressed()
{
    CGlobal::key_value = 3;
}

void stellax_gui_node::MainWindow::on_btn_left_released()
{
    CGlobal::key_value = -1;
}

void stellax_gui_node::MainWindow::on_btn_right_pressed()
{
    CGlobal::key_value = 4;
}

void stellax_gui_node::MainWindow::on_btn_right_released()
{
    CGlobal::key_value = -1;
}

void stellax_gui_node::MainWindow::on_btn_stop_pressed()
{
    CGlobal::key_value = 0;
}

void stellax_gui_node::MainWindow::on_btn_stop_released()
{
    CGlobal::key_value = 0;
}

void stellax_gui_node::MainWindow::on_btn_keyboard_teleop_clicked()
{
    static bool flag_keyboard_teleop = true;
        if (flag_keyboard_teleop) {
            ui.btn_keyboard_teleop->setStyleSheet("background-color: rgb(255,0,255);");
            ui.btn_keyboard_teleop->setText("close Keyboard Teleop");
            system("gnome-terminal -x bash -c 'source ~/stellax_ws/devel/setup.bash;"
                   "roslaunch stellax_serial stellax_keyboard_teleop.launch limited:=true'&");
            flag_keyboard_teleop = false;
        } else {
            ui.btn_keyboard_teleop->setText("Keyboard Teleop");
            ui.btn_keyboard_teleop->setStyleSheet("background-color: rgb(202,235,216);");
            system("killall -g roslaunch stellax_serial stellax_keyboard_teleop.launch");
            flag_keyboard_teleop = true;
        }
}

void stellax_gui_node::MainWindow::on_btn_make_a_map_clicked()
{
    static bool flag_make_a_map = true;
        if (flag_make_a_map) {
            ui.btn_make_a_map->setStyleSheet("background-color: rgb(255,0,255);");
            ui.btn_make_a_map->setText("close Make A Map");
            system("gnome-terminal -x bash -c 'source ~/stellax_ws/devel/setup.bash;"
                   "roslaunch stellax_navigation stellax_make_a_map.launch limited:=true'&");
            flag_make_a_map = false;
        } else {
            ui.btn_make_a_map->setStyleSheet("background-color: rgb(202,235,216);");
            ui.btn_make_a_map->setText("Make A Map");
            system("killall -g roslaunch stellax_navigation stellax_make_a_map.launch");
            flag_make_a_map = true;
        }
}

void stellax_gui_node::MainWindow::on_btn_save_a_map_clicked()
{
    static bool flag_save_a_map = true;
        if (flag_save_a_map) {
            ui.btn_save_a_map->setStyleSheet("background-color: rgb(255,0,255);");
            ui.btn_save_a_map->setText("close Save A Map");
            system("gnome-terminal -x bash -c 'source /opt/ros/indigo/setup.bash;"
                   "rosrun map_server map_saver -f /tmp/my_map; limited:=true'&");
            flag_save_a_map = false;
        } else {
            ui.btn_save_a_map->setStyleSheet("background-color: rgb(202,235,216);");
             ui.btn_save_a_map->setText("Save A Map");
            system("killall -g rosrun map_server map_saver");
            flag_save_a_map = true;
        }
}

void stellax_gui_node::MainWindow::on_btn_navigation_clicked()
{
    static bool flag_navigation = true;
        if (flag_navigation) {
            ui.btn_navigation->setStyleSheet("background-color: rgb(255,0,255);");
            ui.btn_navigation->setText("close Navigation");
            system("gnome-terminal -x bash -c 'source ~/stellax_ws/devel/setup.bash;"
                   "roslaunch stellax_navigation stellax_navigation.launch; limited:=true'&");
            flag_navigation = false;
        } else {
            ui.btn_navigation->setStyleSheet("background-color: rgb(202,235,216);");
            ui.btn_navigation->setText("Navigation");
            system("killall -g roslaunch stellax_navigation stellax_navigation.launch");
            flag_navigation = true;
        }
}

void stellax_gui_node::MainWindow::on_btn_person_follower_clicked()
{
    static bool flag_person_follower = true;
        if (flag_person_follower) {
            ui.btn_person_follower->setStyleSheet("background-color: rgb(255,0,255);");
            ui.btn_person_follower->setText("close Person Follower");
            system("gnome-terminal -x bash -c 'source ~/stellax_ws/devel/setup.bash;"
                   "roslaunch stellax_serial stellax_serial.launch; limited:=true'&");
            QTime t;
            t.start();
            while(t.elapsed() < 500);
            system("gnome-terminal -x bash -c 'source /opt/ros/indigo/setup.bash;"
                   "roslaunch freenect_launch freenect-registered-xyzrgb.launch; limited:=true'&");
            t.start();
            while(t.elapsed() < 500);
            system("gnome-terminal -x bash -c 'source ~/rbx_ws/devel/setup.bash;"
                   "roslaunch rbx1_apps follower2.launch; limited:=true'&");
            flag_person_follower = false;
        } else {
            ui.btn_person_follower->setStyleSheet("background-color: rgb(202,235,216);");
            ui.btn_person_follower->setText("Person Follower");
            system("killall -g roslaunch rbx1_apps follower2.launch");
            system("killall -g roslaunch freenect_launch freenect-registered-xyzrgb.launch");
            system("killall -g roslaunch stellax_serial stellax_serial.launch");
            flag_person_follower = true;
        }
}
