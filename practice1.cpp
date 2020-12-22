#include "3ExtractObject.h"

//布尔型函数 - 深度图存储 - 与读取配套使用
bool ThreeEO::depthimg_save2(const cv::Mat& depimg, std::string path)
{
    cv::Mat saveimg = cv::Mat(depimg.size(), CV_8UC3);
    for (int i = 0; i < depimg.rows; ++i)
    {
        const short int* dep = depimg.ptr<short int>(i);
        uchar* save = saveimg.ptr<uchar>(i);
        for (int j = 0; j < depimg.cols; ++j)
        {
            *save++ = static_cast<uchar>((*dep >> 8) & 0x00ff);
            *save++ = static_cast<uchar>(*dep++ & 0x00ff);
            *save++ = 0;
        }
    }
    cv::imwrite(path, saveimg);
    return true;
}

//布尔型函数 - 深度图读取 - 与存储配套使用
bool ThreeEO::depthimg_read2(const std::string path, cv::Mat& depimg)
{
    cv::Mat saveimg = cv::imread(path);
    depimg = cv::Mat(saveimg.size(), CV_16UC1);
    for (int i = 0; i < depimg.rows; ++i)
    {
        const uchar* save = saveimg.ptr<uchar>(i);
        unsigned short int* dep = depimg.ptr<unsigned short int>(i);
        for (int j = 0; j < depimg.cols; ++j)
        {
            *dep++ = static_cast<unsigned short int>((*save << 8) | (*(save + 1)));
            save += 3;
        }
    }
    return true;
}

int ThreeEO::GetRandomNumber()
{

    int RandomNumber;
    RandomNumber = rand() % (256) + 0;//0到255之间选择颜色
    //生成其他范围的数字：RandomNumber = rand() % (b-a+1) + a;
    return RandomNumber;
}

void ThreeEO::get_planars_by_region_growing(cv::Mat depImg,cv::Mat bgrImg)
{
    start = clock();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //use YUV image to filter side plate
    Mat yuvimg;
    cvtColor(bgrImg, yuvimg, COLOR_BGR2YUV);

    //if (pcl::io::loadPCDFile <pcl::PointXYZ>("region_growing_tutorial.pcd", *cloud) == -1)
    //{
    //    std::cout << "Cloud reading failed." << std::endl;
    //    return (-1);
    //}
    // depth是16UC1的单通道图像
    // 遍历深度图
    for (int i = 200; i < 1100; i = i + 5)
        for (int j = 100; j < 600; j = j + 5)
        {
            // 获取深度图中(i,j)处的值
            unsigned short int d = depImg.at<unsigned short int>(j, i);
            // d 可能没有值，若如此，跳过此点
            if (d == 0 || d < suppres_min || d > suppres_max
                 || (yuvimg.at<Vec3b>(j, i)[0] >= yuvmin[0] && yuvimg.at<Vec3b>(j, i)[0] < yuvmax[0]
                && yuvimg.at<Vec3b>(j, i)[1] >= yuvmin[1] && yuvimg.at<Vec3b>(j, i)[1] < yuvmax[1]
                && yuvimg.at<Vec3b>(j, i)[2]> yuvmin[2] && yuvimg.at<Vec3b>(j, i)[2] < yuvmax[2]))
                continue;
            // d 符合条件，则向点云增加一个点

            pcl::PointXYZ p;
            // 计算这个点的空间坐标
            p.z = double(d) / camera_factor;
            p.x = (double(i) - camera_cx) * p.z / camera_fx;
            p.y = (double(j) - camera_cy) * p.z / camera_fy;

            // 把p加入到点云中
            cloud->points.push_back(p);
            //cout << cloud->points.size() << endl;
        }

    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << "point cloud size = " << cloud->points.size() << endl;
    cloud->is_dense = false;


    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(25);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices(new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    //设置可索引的点（初步过滤条件，其实可以省略，直接在深度图转化中就过滤掉）
    //pass.setFilterFieldName("z");
    //pass.setFilterLimits(400.0, 700.0);
    //pass.filter(*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(400);//设置簇中点的最小值
    reg.setMaxClusterSize(100000);//设置簇中点的最大值
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud);
    //reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(4.5 / 180.0 * M_PI);//点法线角度容差
    reg.setCurvatureThreshold(2.5);//点曲率容差

    //启动分割算法，返回clusters数组
    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);
    
    //TODO:对分割好的每一个簇进行平面拟合(除去与xoy平面垂直的面)
    //再选出最符合中心点的面(中心点基本上在一个范围之内）
    ctrpointpx.clear();
    for (int i = 0; i < clusters.size(); i++)
    {
        cout << clusters[i].indices.size() << endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxx(new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j < clusters[i].indices.size(); j++)
        {
            cloudxx->points.push_back(cloud->points[clusters[i].indices[j]]);
        }
        get_planars2(*cloudxx, i);
    }

    decidecp();
    //circle(depImg_rgb, Point2i(int(ctrpointpx[0].x), int(ctrpointpx[0].y)), 2, Scalar(0, 0, 255), -1);
    //imshow("center", depImg_rgb);
    circle(bgrImg, Point2i(int(ctrpointpx[0].x), int(ctrpointpx[0].y)), 2, Scalar(0, 0, 255), -1);
    imshow("center", bgrImg);
    waitKey(0);
    //教程，说明了怎么样访问点
    //std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
    //std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
    //std::cout << "These are the indices of the points of the initial" <<
    //    std::endl << "cloud that belong to the first cluster:" << std::endl;
    //int counter = 0;
    //while (counter < clusters[0].indices.size())
    //{
    //    std::cout << clusters[0].indices[counter] << ", ";
    //    counter++;
    //    if (counter % 10 == 0)
    //        std::cout << std::endl;
    //}
    //std::cout << std::endl;

    /***/
    //随机为每一个分割好的面分配颜色（RNG）,只能保证属于同一簇的颜色相同，不同簇的颜色有可能相同。
    //红色的地方，说明不在任何一个簇
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::PCLVisualizer viewer("planar segment", true);
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud, "sample");//将single_color在viewer中进行显示
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample");
    /***/


    end = clock();
    endtime = double(end - start) / CLOCKS_PER_SEC;
    cout << "视觉部分耗时:" << endtime << "s" << endl;

    /***/
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(1);
    }
    /***/

}
void ThreeEO::get_planars2(pcl::PointCloud<pcl::PointXYZ> cloud_xx,int i)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloud_xx.makeShared();
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar_segment(new pcl::PointCloud<pcl::PointXYZ>);//创建分割对象
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//模型系数
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//索引列表
    pcl::SACSegmentation<pcl::PointXYZ> seg;//分割对象
    /***/
    pcl::visualization::PCLVisualizer viewer("planar segment", true);
    viewer.setBackgroundColor(0, 0, 0);
    /***/
    pcl::ExtractIndices<pcl::PointXYZ> extract;//提取器

    int n_piece = 1;//需要探测的面的个数

    //使用RANSAC获取点数最多的面

    seg.setOptimizeCoefficients(true);		//使用内部点重新估算模型参数
    seg.setModelType(pcl::SACMODEL_PLANE);   //设置模型类型
    seg.setMethodType(pcl::SAC_RANSAC);      //设置随机采样一致性方法类型
    seg.setDistanceThreshold(6);    //设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    //提取探测出来的平面
    extract.filter(*planar_segment);
    //planar_segment为该次探测出来的面片，可以单独进行保存，此处省略

    //剔除探测出的平面，在剩余点中继续探测平面
    extract.setNegative(true);
    extract.filter(*cloud);

    /***/
    //可视化面
    int R = GetRandomNumber();
    int G = GetRandomNumber();
    int B = GetRandomNumber();
    stringstream ss;
    ss << i + 1;
    string str;
    ss >> str;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(planar_segment, R, G, B);
    viewer.addPointCloud<pcl::PointXYZ>(planar_segment, single_color, str);//将single_color在viewer中进行显示
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, str);
    /***/


    //计算角度
    double a = coefficients->values[0] / coefficients->values[3];
    double b = coefficients->values[1] / coefficients->values[3];
    double c = coefficients->values[2] / coefficients->values[3];

    double thetaz;//(与xoy平面的夹角）
    thetaz = acos(c / sqrt(a * a + b * b + c * c)) * 180.0 / 3.1415925;
    
    //cout << planar_segment->size() << endl;

    if (thetaz > 80 && thetaz < 100)
    {
        return;
    }
    
    else if(planar_segment->size() > 500)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*planar_segment, centroid);
        //点云中心可视化
        pcl::PointXYZ center;
        center.x = centroid[0];
        center.y = centroid[1];
        center.z = centroid[2];

        pixelx = center.x * camera_fx / center.z + camera_cx;
        pixely = center.y * camera_fy / center.z + camera_cy;
        //circle(bgrImg, Point(pixelx, pixely), 3, Scalar(0, 0, 255), -1);
        //imshow("center", bgrImg);
        //waitKey(0);
        ctrpointpx.push_back(Point2f(pixelx, pixely));
        cout << "第" << i << "个面的点个数：" << planar_segment->size() << endl;
        cout << "第" << i << "个面的角度为：" << thetaz << endl;
        //cout << "第" << i << "个面的中心为：(" << centroid[0] << "," << centroid[1] << "," << centroid[2] << ")" << endl;
        cout << "第" << i << "个面的pixel中心为：(" << pixelx << "," << pixely << "," << centroid[2] << ")" << endl;

        /***/
        //可视化中心点
        stringstream ss;
        ss << i + 10;
        string str;
        ss >> str;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ctr(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_ctr->points.push_back(center);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_ctr, 255, 0, 0);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_ctr, single_color, str);//将single_color在viewer中进行显示
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, str);
        /***/

        //返回
        ctrp.x = centroid[0];
        ctrp.y = centroid[1];
        /*TODO:这个z直接返回深度图像求取*/
        ctrp.z = centroid[2];
        angle = thetaz;
    }
  
    /***/
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(1);
    }
    /***/
}

void ThreeEO::decidecp()
{
    if (ctrpointpx.size() == 1)
        return;
    //直接计算离图像中心点的欧式距离就好
    double cx = depImg.cols / 2, cy = depImg.rows / 2;
    int n = ctrpointpx.size();
    double min_dis = 10000000;
    int min_index = 0;
    for (int i = 0; i < n; i++)
    {
        double temp = pow((ctrpointpx[i].x - cx), 2) + pow((ctrpointpx[i].y - cy), 2);
        if (min_dis > temp)
        {
            min_dis = temp;
            min_index = i;
        }
    }
    Point2f temp;
    temp = ctrpointpx[min_index];
    ctrpointpx.clear();
    ctrpointpx.push_back(temp);

    ctrp.x = temp.x;
    ctrp.y = temp.y;
    pixelx = int(temp.x);
    pixely = int(temp.y);
    ctrp.z = depImg.at<unsigned short int>(pixely, pixelx);
}

void ThreeEO::get_planars_by_color(cv::Mat depImg, cv::Mat bgrImg)
{
    start = clock();

    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    //if (pcl::io::loadPCDFile <pcl::PointXYZRGB>("region_growing_rgb_tutorial.pcd", *cloud) == -1)
    //{
    //    std::cout << "Cloud reading failed." << std::endl;
    //    return;
    //}
    // 相机内参
    const double camera_factor = 1;
    const double camera_cx = 1280 / 2;
    const double camera_cy = 720 / 2;
    const double camera_fx = 915.539;
    const double camera_fy = 913.727;

    // depth是16UC1的单通道图像
    // 遍历深度图
    for (int i = 0; i < depImg.cols; i = i + 5)
        for (int j = 0; j < depImg.rows; j = j + 5)
        {
            // 获取深度图中(i,j)处的值
            unsigned short int d = depImg.at<unsigned short int>(j, i);
            // d 可能没有值，若如此，跳过此点
            if (d == 0 || d < 400 || d > 700)
                continue;
            // d 符合条件，则向点云增加一个点

            pcl::PointXYZRGB p;
            // 计算这个点的空间坐标
            p.z = double(d) / camera_factor;
            p.x = (double(i) - camera_cx) * p.z / camera_fx;
            p.y = (double(j) - camera_cy) * p.z / camera_fy;
            p.b = bgrImg.at<Vec3b>(j, i)[0];
            p.g = bgrImg.at<Vec3b>(j, i)[1];
            p.r = bgrImg.at<Vec3b>(j, i)[2];
            // 把p加入到点云中
            cloud->points.push_back(p);
            //cout << cloud->points.size() << endl;
        }

    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << "point cloud size = " << cloud->points.size() << endl;
    cloud->is_dense = false;
    
    //try {
    //    //保存点云图
    //    pcl::io::savePCDFile("./data/pcd.pcd", *cloud);
    //}
    //catch (pcl::IOException& e) {
    //    cout << e.what() << endl;
    //}
    //cloud->points.clear();

    //if (pcl::io::loadPCDFile <pcl::PointXYZRGB>("./data/pcd.pcd", *cloud) == -1)
    //{
    //    std::cout << "Cloud reading failed." << std::endl;
    //    return;
    //}
    pcl::IndicesPtr indices(new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");//以z轴为分割标准
    pass.setFilterLimits(400.0, 700.0);//只关心z轴的值在400-700之间的区域
    pass.filter(*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud);
    reg.setIndices(indices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(10);//邻居之间的最大距离
    reg.setPointColorThreshold(6);//点之间颜色的容差值
    reg.setRegionColorThreshold(5);//区域之间的容差
    reg.setMinClusterSize(600);//设置聚类的大小

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::PCLVisualizer viewer("planar segment", true);
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud, "sample");//将single_color在viewer中进行显示
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample");
    
    end = clock();
    endtime = double(end - start) / CLOCKS_PER_SEC;
    cout << "视觉部分耗时:" << endtime << "s" << endl;

    //viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(1);
        //std::this_thread::sleep_for(100us);
    }

}
void ThreeEO::get_planars(cv::Mat depth)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 相机内参
    const double camera_factor = 1;
    const double camera_cx = 1280 / 2;
    const double camera_cy = 720 / 2;
    const double camera_fx = 915.539;
    const double camera_fy = 913.727;

    // depth是16UC1的单通道图像
    // 遍历深度图
    for (int i = 0; i < depth.cols; i = i + 10)
        for (int j = 0; j < depth.rows; j = j + 10)
        {
            // 获取深度图中(i,j)处的值
            unsigned short int d = depth.at<unsigned short int>(j, i);
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点

            pcl::PointXYZ p;
            // 计算这个点的空间坐标
            p.z = double(d) / camera_factor;
            p.x = (double(i) - camera_cx) * p.z / camera_fx;
            p.y = (double(j) - camera_cy) * p.z / camera_fy;

            // 把p加入到点云中
            cloud->points.push_back(p);
            //cout << cloud->points.size() << endl;
        }

    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << "point cloud size = " << cloud->points.size() << endl;
    cloud->is_dense = false;

    //第一步：定义输入的原始数据及滤波后的点，以及分割获得的点、平面系数coefficients、存储内点的索引集合对象inliers、用于显示的窗口
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar_segment(new pcl::PointCloud<pcl::PointXYZ>);//创建分割对象
    //pcl::io::loadPCDFile("projectpointcloud.pcd", *cloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//模型系数
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//索引列表
    pcl::SACSegmentation<pcl::PointXYZ> seg;//分割对象
    pcl::visualization::PCLVisualizer viewer("planar segment",true);
    viewer.setBackgroundColor(0, 0, 0);
    pcl::ExtractIndices<pcl::PointXYZ> extract;//提取器

    int n_piece = 6;//需要探测的面的个数
    vector<double> thetas;
    vector<vector<double>> abcs(n_piece);
    //当角度相似时，记录最大的平面
    vector<pcl::PointCloud<pcl::PointXYZ>> planars;
    //vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planars(n_piece);
    //for (int i = 0;i < n_piece;i++)
    //{
    //pcl::PointCloud<pcl::PointXYZ>::Ptr p1(new pcl::PointCloud<pcl::PointXYZ>);
    //planars.push_back(p1);
    //}

    //第二步：将原始点加载进入(红色的)
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
    //viewer.addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample");//将single_color在viewer中进行显示
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample");
    //第三步：使用RANSAC获取点数最多的面
    for (int i = 0; i < n_piece; i++)
    {
        seg.setOptimizeCoefficients(true);		//使用内部点重新估算模型参数
        seg.setModelType(pcl::SACMODEL_PLANE);   //设置模型类型
        //seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);   //设置模型类型
        seg.setMethodType(pcl::SAC_RANSAC);      //设置随机采样一致性方法类型
        seg.setDistanceThreshold(0.1);    //设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        //提取探测出来的平面
        extract.filter(*planar_segment);
        //planar_segment为该次探测出来的面片，可以单独进行保存，此处省略

        //剔除探测出的平面，在剩余点中继续探测平面
        extract.setNegative(true);
        extract.filter(*cloud);


        int R = r[i];
        int G = g[i];
        int B = b[i];
        //int R = GetRandomNumber();
        //int G = GetRandomNumber();
        //int B = GetRandomNumber();
        stringstream ss;
        ss << i + 1;
        string str;
        ss >> str;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(planar_segment, R, G, B);
        viewer.addPointCloud<pcl::PointXYZ>(planar_segment, single_color, str);//将single_color在viewer中进行显示
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, str);


            //计算角度
        double a = coefficients->values[0] / coefficients->values[3];
        double b = coefficients->values[1] / coefficients->values[3];
        double c = coefficients->values[2] / coefficients->values[3];

        double theta;
        theta = acos(a / sqrt(a * a + b * b + c * c)) * 180.0 / 3.1415925;
        //cout << planar_segment->size() << endl;
        //cout << theta << endl;

        //将角度放入向量
        thetas.push_back(theta);
        //planars[i] = planar_segment;
        planars.push_back(*planar_segment);

        if (thetas.size() >= 2)
        {
            int k = thetas.size() - 1;
            for (int j = 0; j < k; j++)
            {
                if (abs(thetas[j] - thetas[k]) < 5.0 && planars[j].size() != 0)
                {
                    planars[k] = planars[k] + planars[j];
                    //cout << planars[k].size() << endl;
                    planars[j].clear();
                }
            }
        }

    }

    for (int i = 0; i < n_piece; i++)
    {
        if (planars[i].size() != 0)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(planars[i], centroid);

            //点云中心可视化
            pcl::PointXYZ center;
            center.x = centroid[0];
            center.y = centroid[1];
            center.z = centroid[2];

            pixelx = center.x * camera_fx / center.z + camera_cx;
            pixely = center.y * camera_fy / center.z + camera_cy;
            cout << "第" << i << "个面的点个数：" << planars[i].size() << endl;
            cout << "第" << i << "个面的角度为：" << thetas[i] << endl;
            //cout << "第" << i << "个面的中心为：(" << centroid[0] << "," << centroid[1] << "," << centroid[2] << ")" << endl;
            cout << "第" << i << "个面的pixel中心为：(" << pixelx << "," << pixely << "," << centroid[2] << ")" << endl;

            stringstream ss;
            ss << i + 10;
            string str;
            ss >> str;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ctr(new pcl::PointCloud<pcl::PointXYZ>);
            cloud_ctr->points.push_back(center);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_ctr, 255, 0, 0);
            viewer.addPointCloud<pcl::PointXYZ>(cloud_ctr, single_color, str);//将single_color在viewer中进行显示
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, str);

                  //返回
            ctrp.x = centroid[0];
            ctrp.y = centroid[1];
            ctrp.z = centroid[2];
            angle = thetas[i];
        }
    }

    while (!viewer.wasStopped())
    {
      viewer.spinOnce(1);
    }
}
//todo:不要用点云计算中心点的深度，直接返回深度图找深度
