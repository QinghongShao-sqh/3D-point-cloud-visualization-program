// by kk nuaa 
#include "mainwindow.h"

#include <QApplication>

#include <vtkOutputWindow.h>
int main(int argc, char *argv[])
{
	vtkOutputWindow::SetGlobalWarningDisplay(0);//不弹出vtkOutputWindow窗口
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
