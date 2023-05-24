// by kk nuaa 

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog.h>
#include <QMessageBox.h>



MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //点云初始化

	m_Cloud.reset(new PointCloudT);

	//可视化对象初始化
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));

	//设置VTK可视化窗口指针
	ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());

	//设置窗口交互，窗口可接受键盘等事件
	viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());

	ui->qvtkWidget->update();

}

MainWindow::~MainWindow()
{
    delete ui;
}

//打开点云
void MainWindow::on_actionOpen_triggered()
{

	//获取点云路径
	QString path = QFileDialog::getOpenFileName(this, "选择点云", ".//", "点云文件(*.ply *.pcd);;所有文件(*.*)");

	PointCloudT::Ptr cloud_tmp(new PointCloudT);

	if (path.isEmpty())
		return;

	int return_status;
	if (path.endsWith(".pcd", Qt::CaseInsensitive))
	{
		return_status = pcl::io::loadPCDFile(path.toStdString(), *cloud_tmp);
	}		
	else if (path.endsWith(".ply", Qt::CaseInsensitive))
	{
		return_status = pcl::io::loadPLYFile(path.toStdString(), *cloud_tmp);
	}
	else
	{
		return;
	}

	if (return_status != 0)
	{
		return;
	}

	//清空点云
	m_Cloud->clear();
	viewer->removeAllPointClouds();
	viewer->removeAllCoordinateSystems();

	if (cloud_tmp->is_dense)
		pcl::copyPointCloud(*cloud_tmp, *m_Cloud);
	else
	{
		PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
		std::vector<int> vec;
		pcl::removeNaNFromPointCloud(*cloud_tmp, *m_Cloud, vec);
	}

	//添加到窗口
	viewer->addPointCloud(m_Cloud);

	pcl::getMinMax3D(*m_Cloud, p_min, p_max);
	maxLen = getMaxValue(p_max, p_min);

	//重设视角
	viewer->resetCamera();

	//刷新窗口
	ui->qvtkWidget->update();
}


//俯视图
void MainWindow::on_actionUp_triggered()
{



	if (!m_Cloud->empty())
	{
		viewer->setCameraPosition(0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), p_max.z + 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), p_max.z, 0, 1, 0);
		ui->qvtkWidget->update();
	}
}

//底视图
void MainWindow::on_actionBottom_triggered()
{
	if (!m_Cloud->empty())
	{
		viewer->setCameraPosition(0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), p_min.z - 2 * maxLen, 0.5*(p_min.x + p_max.x), 0.5*(p_min.y + p_max.y), p_min.z, 0, 1, 0);
		ui->qvtkWidget->update();
	}
}

//前视图
void MainWindow::on_actionFront_triggered()
{
	if (!m_Cloud->empty())
	{
		viewer->setCameraPosition(0.5*(p_min.x + p_max.x), p_min.y - 2 * maxLen, 0.5*(p_min.z + p_max.z), 0.5*(p_min.x + p_max.x), p_min.y, 0.5*(p_min.z + p_max.z), 0, 0, 1);
		ui->qvtkWidget->update();
	}
}

//后视图
void MainWindow::on_actionBack_triggered()
{
	if (!m_Cloud->empty())
	{
		viewer->setCameraPosition(0.5*(p_min.x + p_max.x), p_max.y + 2 * maxLen, 0.5*(p_min.z + p_max.z), 0.5*(p_min.x + p_max.x), p_min.y, 0.5*(p_min.z + p_max.z), 0, 0, 1);
		ui->qvtkWidget->update();
	}
}

//左视图
void MainWindow::on_actionLeft_triggered()
{
	if (!m_Cloud->empty())
	{
		viewer->setCameraPosition(p_min.x - 2 * maxLen, 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), p_max.x, 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 0, 0, 1);
		ui->qvtkWidget->update();
	}
}



//右视图
void MainWindow::on_actionRight_triggered()
{
	if (!m_Cloud->empty())
	{
		viewer->setCameraPosition(p_max.x + 2 * maxLen, 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), p_max.x, 0.5*(p_min.y + p_max.y), 0.5*(p_min.z + p_max.z), 0, 0, 1);
		ui->qvtkWidget->update();
	}
}


double MainWindow::getMinValue(PointT p1, PointT p2)
{
	double min = 0;

	if (p1.x - p2.x > p1.y - p2.y)
	{
		min = p1.y - p2.y;
	}
	else
	{
		min = p1.x - p2.x;
	}

	if (min > p1.z - p2.z)
	{
		min = p1.z - p2.z;
	}

	return min;
}


double MainWindow::getMaxValue(PointT p1, PointT p2)
{
	double max = 0;

	if (p1.x - p2.x > p1.y - p2.y)
	{
		max = p1.x - p2.x;

	}
	else
	{
		max = p1.y - p2.y;
	}

	if (max < p1.z - p2.z)
	{
		max = p1.z - p2.z;
	}

	return max;
}