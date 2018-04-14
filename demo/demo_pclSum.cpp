#include "pclHeader.h"

int main(int argc, char** argv)
{
    //asc2pcd("points.asc");
	const float depth_limit_z_min = 1.4;
	const float depth_limit_z_max = 2.0;
	const float depth_limit_x_min = -0.1;
	const float depth_limit_x_max = 0.6;
	const float depth_limit_y_min = -0.7;
	const float depth_limit_y_max = 0.4;
  
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempx_cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempy_cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempz_cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）
    
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("points.pcd", *cloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
        return (-1);
    }
	//依据距离过滤点云
	pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (depth_limit_z_min,depth_limit_z_max);
    pass.filter (*tempz_cloud);
	cout<< "z方向滤波结束"<<endl;
	cloud = tempz_cloud;
	
	pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (depth_limit_x_min,depth_limit_x_max);
    pass.filter (*tempx_cloud);
	cout<< "x方向滤波结束"<<endl;
	cloud = tempx_cloud;
	
	pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (depth_limit_y_min,depth_limit_y_max);
    pass.filter (*tempy_cloud);
	cout<< "x方向滤波结束"<<endl;
	cloud = tempy_cloud;
	
	//voxel滤波简化
	pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
	// pcl::VoxelGrid filter 的 leaf size 为 1cm，结果储存在cloud_downsampled中
	voxelSampler.setInputCloud(cloud);
	voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
	voxelSampler.filter(*cloud_downsampled);
	cout<< "voxel滤波结束"<<endl;
	cloud = cloud_downsampled;
	
	//StatisticalOutlierRemoval滤波器移除离群点
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;     // 创建滤波器对象
	statFilter.setInputCloud(cloud);//设置呆滤波的点云
	statFilter.setMeanK(10);//设置在进行统计时考虑查询点邻近点数
	statFilter.setStddevMulThresh(0.2);//设置判断是否为离群点的阈值
	statFilter.filter(*cloud_filtered);//执行滤波处理保存内点到cloud_filtered
	cout<< "离群点滤波结束"<<endl;
	cloud = cloud_filtered;

	//直接从点云数据集中近似推断表面法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;    //创建法向估计对象
	ne.setInputCloud(cloud);//为法向估计对象输入点
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);   //设置法向估计时采用的搜索方式为kdtree
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); 
	ne.setRadiusSearch(0.05);   // Use all neighbors in a sphere of radius 1cm 
	ne.compute(*normals);     //计算法向量
	std::cerr << "法线计算完成" << std::endl;

	
	
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals //将点云和法线放到一起
	// Create search tree* //创建搜索树
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals); //利用点云构建搜索树

	// Initialize objects //初始化GreedyProjectionTriangulation对象，并设置参数
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;    //创建多变形网格，用于存储结果
	
	//设置参数特征值和实际三角化
	// Set the maximum distance between connected points (maximum edge length)
	//设置GreedyProjectionTriangulation对象的参数  //第一个参数影响很大
	gp3.setSearchRadius(0.025);    //设置连接点之间的最大距离（三角形最大边长）用于确定k近邻的球半径【默认值 0】
	gp3.setMu(2.5); //设置最近邻距离的乘子，以得到每个点的最终搜索半径【默认值 0】
	gp3.setMaximumNearestNeighbors(100);    //设置搜索的最近邻点的最大数量
	gp3.setMaximumSurfaceAngle(M_PI / 4);      // 45 degrees（pi）最大平面角
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees三角形内角最小角度
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees三角形内角最大角度
	gp3.setNormalConsistency(false);   //设置该参数保证法线朝向一致，如果法向量一致，设置为true

	// Get result
	//设置搜索方法和输入点云
	gp3.setInputCloud(cloud_with_normals);  //设置输入点云为有向点云cloud_with_normals
	gp3.setSearchMethod(tree2);  //设置搜索方式为tree2
	gp3.reconstruct(triangles);	//执行重构，结果保存在triangles中
	
	//保存网格图
	pcl::io::savePLYFile("result.ply", triangles);
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	// Finish结束

	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0); //设置背景
	viewer->addPolygonMesh(triangles, "my"); //设置显示的网格
	//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1,0); //设置坐标系
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{

		viewer->spinOnce(100);

		boost::this_thread::sleep(boost::posix_time::microseconds(100000));

	}
    return (0);
}