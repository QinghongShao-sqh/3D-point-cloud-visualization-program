// by kk nuaa 
#pragma once
//设置中文编码
#pragma execution_character_set("utf-8")


#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <QMainWindow>

namespace Ui { class MainWindow; }

#include <pcl/common/common.h>
#include <pcl/point_types.h>		
#include <pcl/point_cloud.h>				
#include <pcl/visualization/pcl_visualizer.h>	
#include <vtkRenderWindow.h>	
#include <pcl/filters/filter.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


    	//当前的点云
	PointCloudT::Ptr m_Cloud;

	//可视化窗口类
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	//点云坐标极值
	PointT p_min, p_max;

	double maxLen;

	double getMinValue(PointT p1, PointT p2);
	double getMaxValue(PointT p1, PointT p2);
public slots:
	
	void on_actionOpen_triggered();
    
    //俯视图
	void on_actionUp_triggered();

	//底视图
	void on_actionBottom_triggered();

	//前视图
	void on_actionFront_triggered();


	//后视图
	void on_actionBack_triggered();

    //左视图
	void on_actionLeft_triggered();

	//右视图
	void on_actionRight_triggered();


private:
    Ui::MainWindow *ui;
};
