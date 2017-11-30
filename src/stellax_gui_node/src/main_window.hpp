/**
 * @file /include/stellax_gui_node/main_window.hpp
 *
 * @brief Qt based gui for stellax_gui_node.
 *
 * @date November 2010
 **/
#ifndef stellax_gui_node_MAIN_WINDOW_H
#define stellax_gui_node_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace stellax_gui_node {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    void on_btn_start_roscore_clicked();

    void on_btn_up_pressed();

    void on_btn_up_released();

    void on_btn_down_pressed();

    void on_btn_down_released();

    void on_btn_left_pressed();

    void on_btn_left_released();

    void on_btn_right_pressed();

    void on_btn_right_released();

    void on_btn_stop_pressed();

    void on_btn_stop_released();

    void on_btn_keyboard_teleop_clicked();

    void on_btn_make_a_map_clicked();

    void on_btn_save_a_map_clicked();

    void on_btn_navigation_clicked();

    void on_btn_person_follower_clicked();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace stellax_gui_node

#endif // stellax_gui_node_MAIN_WINDOW_H
