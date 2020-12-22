#include "3ExtractObject.h"
#include "HSV_Segmentation.h"
//int yuan3()
int main()
{
	ThreeEO eo;
	Mat bgrImg;


	//int n = 8;//待检测图片的张数
	//for (int i = 8; i <= n; i++)
	//{
	//	string path = "./9.15/" + to_string(i) + ".bmp";
	//	string path_bgr = "./9.15/" + to_string(i) + ".jpg";
	//	cout << path << endl;
	//	eo.depthimg_read2(path, eo.depImg);
	//	eo.depImg_rgb = imread(path);
	//	eo.bgrImg = imread(path_bgr);
	//	eo.get_planars_by_region_growing(eo.depImg);
	//	cout << endl;
	//}
	
	//string path = "./9.16/4.bmp";
	//string path_rgb = "./9.16/4.jpg";
	//eo.depthimg_read2(path, eo.depImg);
	//eo.depImg_rgb = imread(path);
	//eo.bgrImg = imread(path_rgb);
	//eo.suppres_max = 560;
	//eo.suppres_min = 350;
	////eo.suppres_max = 650;
	////eo.suppres_min = 450;
	//eo.get_planars_by_region_growing(eo.depImg);

	vector <vector<int>> suppress = { {470,650},{470,650},{490,650},{410,560},{400,560},{470,650},{500,650},{490,700} };
	int n = 8;//待检测图片的张数
    for (int i = 1; i <= n; i++)
    {
    	string path = "./9.15/" + to_string(i) + ".bmp";
    	string path_bgr = "./9.15/" + to_string(i) + ".jpg";
    	cout << path << endl;
    	eo.depthimg_read2(path, eo.depImg);
    	eo.depImg_rgb = imread(path);
    	eo.bgrImg = imread(path_bgr);
		eo.suppres_min = suppress[i-1][0];
		eo.suppres_max = suppress[i-1][1];
    	eo.get_planars_by_region_growing(eo.depImg, eo.bgrImg);
    	cout << endl;
    }

	////calculate acc
	//vector <vector<double>> get_by_pic = { {643,432.5},{613,353},{669.5,357.5},{769.5,405},{531.5,457},{638,365.5},{489,445},{652,404} };
	//vector <vector<double>> get_by_code = { {641.316,438.925},{607.013,355.495},{667.908,361.859},{755.322,412.17},{531.48,460.028},{636.146,375.899},{484.488,454.322},{644.396,415.11} };
	//double avgErr_x = 0, avgErr_y = 0, avgErr_dis = 0;
	//for (int i = 0; i < get_by_pic.size(); i++)
	//{
	//	avgErr_x += abs(get_by_pic[i][0] - get_by_code[i][0]);
	//	avgErr_y += abs(get_by_pic[i][1] - get_by_code[i][1]);
	//	avgErr_dis += sqrt(pow((get_by_pic[i][0] - get_by_code[i][0]), 2) + pow((get_by_pic[i][1] - get_by_code[i][1]), 2));
	//}
	//avgErr_x = avgErr_x / double(get_by_pic.size());
	//avgErr_y = avgErr_y / double(get_by_pic.size());
	//avgErr_dis = avgErr_dis / double(get_by_pic.size());
	//cout << "x方向上的平均误差：" << avgErr_x << endl;
	//cout << "y方向上的平均误差：" << avgErr_y << endl;
	//cout << "欧式距离的平均误差：" << avgErr_dis << endl;

	//对HSV_S.cpp的测试
	//for (int i = 0; i <= 23; i++)
	//{
	//	cout << "第" << i << "张照片" << endl;
	//	string path = "D:\\深圳先进院\\共融比赛2020\\11.06\\" + to_string(i) + "_color.jpg";
	//	HSV_S vv(path);
	//	vv.hsvseg(vv.hsvmin, vv.hsvmax, vv.MinArea, vv.MaxArea, vv.result);
	//	//vv.hsvseg_sperate(vv.hsvmin, vv.hsvmax, vv.hsvmin2, vv.hsvmax2, vv.MinArea, vv.MaxArea, vv.result);
	//	//vv.YUVseg(vv.yuvmin, vv.yuvmax, vv.MinArea, vv.MaxArea, vv.result);
	//}
	//HSV_S vv("D:\\深圳先进院\\共融比赛2020\\11.06\\0_color.jpg");
	//vv.hsvseg(vv.hsvmin, vv.hsvmax, vv.MinArea, vv.MaxArea, vv.result);
	return 0;
}
