//直接拟合平面
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdlib.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
/**********************************/
#include <iostream>
#include <thread>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/features/normal_3d.h>

using namespace std::chrono_literals;
/***********************************/

using namespace std;
using namespace cv;

//直接拟合平面
class ThreeEO
{
private:
	//红，绿，蓝，黄，紫色,青色,白色
	vector<int> r = { 255,0,0,255,255,0,255 };
	vector<int> g = { 0,255,0,255,0,255,255 };
	vector<int> b = { 0,0,255,0,255,255,255 };
	
	//快递架隔板YUV空间颜色
	vector<int>yuvmin = { 160,120,120 };
	vector<int>yuvmax = { 210,130,140 };

public:
	Mat bgrImg;
	Mat depImg;
	Mat depImg_rgb;
	clock_t start, end;
	double endtime;
	int suppres_min = 350, suppres_max = 650; //限定关注的距离，默认值是350到650。每一个格子可能不同。
	//结果返回
	vector<Point2f>ctrpointpx;//经过过滤之后的平面的中心点，过滤的好的话，只有一个；不好的话，可能有多个，对于多个的情况，使用函数decidecp
	double pixelx, pixely;
	cv::Point3d	ctrp = cv::Point3d(0, 0, 0);
	double angle = 0.0;
	// 相机内参
	const double camera_factor = 1;
	const double camera_cx = 1280 / 2;
	const double camera_cy = 720 / 2;
	const double camera_fx = 915.539;
	const double camera_fy = 913.727;

	/*深度图部分*/
	//布尔型函数 - 深度图存储 - 与读取配套使用
	bool depthimg_save2(const cv::Mat& depimg, std::string path);
	//布尔型函数 - 深度图读取 - 与存储配套使用
	bool depthimg_read2(const std::string path, cv::Mat& depimg);
	/*点云部分*/
	int GetRandomNumber();//只是用于平面上色
	void get_planars(cv::Mat depth);//平面拟合
	void get_planars_by_color(cv::Mat depImg, cv::Mat bgrImg);
	void get_planars_by_region_growing(cv::Mat depImg, cv::Mat bgrImg);
	void get_planars2(pcl::PointCloud<pcl::PointXYZ> cloud, int i);
	/*决定多个中心点的情况*/
	void decidecp();

	/*TODO:如果是旋转盒子的情况，拟合两个面，如果面角度相似就合并;否则就输出两个平面的中心点和坐标*/
};
