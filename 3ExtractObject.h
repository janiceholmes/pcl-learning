//ֱ�����ƽ��
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
#include <pcl/sample_consensus/method_types.h>   //����������Ʒ���ͷ�ļ�
#include <pcl/sample_consensus/model_types.h>   //ģ�Ͷ���ͷ�ļ�
#include <pcl/segmentation/sac_segmentation.h>   //���ڲ���һ���Էָ�����ͷ�ļ�
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

//ֱ�����ƽ��
class ThreeEO
{
private:
	//�죬�̣������ƣ���ɫ,��ɫ,��ɫ
	vector<int> r = { 255,0,0,255,255,0,255 };
	vector<int> g = { 0,255,0,255,0,255,255 };
	vector<int> b = { 0,0,255,0,255,255,255 };
	
	//��ݼܸ���YUV�ռ���ɫ
	vector<int>yuvmin = { 160,120,120 };
	vector<int>yuvmax = { 210,130,140 };

public:
	Mat bgrImg;
	Mat depImg;
	Mat depImg_rgb;
	clock_t start, end;
	double endtime;
	int suppres_min = 350, suppres_max = 650; //�޶���ע�ľ��룬Ĭ��ֵ��350��650��ÿһ�����ӿ��ܲ�ͬ��
	//�������
	vector<Point2f>ctrpointpx;//��������֮���ƽ������ĵ㣬���˵ĺõĻ���ֻ��һ�������õĻ��������ж�������ڶ���������ʹ�ú���decidecp
	double pixelx, pixely;
	cv::Point3d	ctrp = cv::Point3d(0, 0, 0);
	double angle = 0.0;
	// ����ڲ�
	const double camera_factor = 1;
	const double camera_cx = 1280 / 2;
	const double camera_cy = 720 / 2;
	const double camera_fx = 915.539;
	const double camera_fy = 913.727;

	/*���ͼ����*/
	//�����ͺ��� - ���ͼ�洢 - ���ȡ����ʹ��
	bool depthimg_save2(const cv::Mat& depimg, std::string path);
	//�����ͺ��� - ���ͼ��ȡ - ��洢����ʹ��
	bool depthimg_read2(const std::string path, cv::Mat& depimg);
	/*���Ʋ���*/
	int GetRandomNumber();//ֻ������ƽ����ɫ
	void get_planars(cv::Mat depth);//ƽ�����
	void get_planars_by_color(cv::Mat depImg, cv::Mat bgrImg);
	void get_planars_by_region_growing(cv::Mat depImg, cv::Mat bgrImg);
	void get_planars2(pcl::PointCloud<pcl::PointXYZ> cloud, int i);
	/*����������ĵ�����*/
	void decidecp();

	/*TODO:�������ת���ӵ��������������棬�����Ƕ����ƾͺϲ�;������������ƽ������ĵ������*/
};
