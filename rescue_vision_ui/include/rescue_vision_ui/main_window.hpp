/**
 * @file /include/rescue_vision_ui/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef rescue_vision_ui_MAIN_WINDOW_H
#define rescue_vision_ui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "std_msgs/String.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rescue_vision_ui
{
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = 0);
  ~MainWindow();
  std::string start_flag = "";

public Q_SLOTS:
  void update();
  void update_thermal();
  void button();

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
};

}  // namespace rescue_vision_ui

#endif  // rescue_vision_ui_MAIN_WINDOW_H
