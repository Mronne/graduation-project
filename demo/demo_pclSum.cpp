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
  
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // �������ƣ�ָ�룩
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempx_cloud(new pcl::PointCloud<pcl::PointXYZ>); // �������ƣ�ָ�룩
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempy_cloud(new pcl::PointCloud<pcl::PointXYZ>); // �������ƣ�ָ�룩
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempz_cloud(new pcl::PointCloud<pcl::PointXYZ>); // �������ƣ�ָ�룩
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>); // �������ƣ�ָ�룩
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>); // �������ƣ�ָ�룩
    
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("points.pcd", *cloud) == -1) //* ����PCD��ʽ���ļ�������ļ������ڣ�����-1
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //�ļ�������ʱ�����ش�����ֹ����
        return (-1);
    }
	//���ݾ�����˵���
	pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (depth_limit_z_min,depth_limit_z_max);
    pass.filter (*tempz_cloud);
	cout<< "z�����˲�����"<<endl;
	cloud = tempz_cloud;
	
	pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (depth_limit_x_min,depth_limit_x_max);
    pass.filter (*tempx_cloud);
	cout<< "x�����˲�����"<<endl;
	cloud = tempx_cloud;
	
	pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (depth_limit_y_min,depth_limit_y_max);
    pass.filter (*tempy_cloud);
	cout<< "x�����˲�����"<<endl;
	cloud = tempy_cloud;
	
	//voxel�˲���
	pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
	// pcl::VoxelGrid filter �� leaf size Ϊ 1cm�����������cloud_downsampled��
	voxelSampler.setInputCloud(cloud);
	voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
	voxelSampler.filter(*cloud_downsampled);
	cout<< "voxel�˲�����"<<endl;
	cloud = cloud_downsampled;
	
	//StatisticalOutlierRemoval�˲����Ƴ���Ⱥ��
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;     // �����˲�������
	statFilter.setInputCloud(cloud);//���ô��˲��ĵ���
	statFilter.setMeanK(10);//�����ڽ���ͳ��ʱ���ǲ�ѯ���ڽ�����
	statFilter.setStddevMulThresh(0.2);//�����ж��Ƿ�Ϊ��Ⱥ�����ֵ
	statFilter.filter(*cloud_filtered);//ִ���˲��������ڵ㵽cloud_filtered
	cout<< "��Ⱥ���˲�����"<<endl;
	cloud = cloud_filtered;

	//ֱ�Ӵӵ������ݼ��н����ƶϱ��淨��
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;    //����������ƶ���
	ne.setInputCloud(cloud);//Ϊ������ƶ��������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);   //���÷������ʱ���õ�������ʽΪkdtree
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); 
	ne.setRadiusSearch(0.05);   // Use all neighbors in a sphere of radius 1cm 
	ne.compute(*normals);     //���㷨����
	std::cerr << "���߼������" << std::endl;

	
	
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals //�����ƺͷ��߷ŵ�һ��
	// Create search tree* //����������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals); //���õ��ƹ���������

	// Initialize objects //��ʼ��GreedyProjectionTriangulation���󣬲����ò���
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;    //����������������ڴ洢���
	
	//���ò�������ֵ��ʵ�����ǻ�
	// Set the maximum distance between connected points (maximum edge length)
	//����GreedyProjectionTriangulation����Ĳ���  //��һ������Ӱ��ܴ�
	gp3.setSearchRadius(0.025);    //�������ӵ�֮��������루���������߳�������ȷ��k���ڵ���뾶��Ĭ��ֵ 0��
	gp3.setMu(2.5); //��������ھ���ĳ��ӣ��Եõ�ÿ��������������뾶��Ĭ��ֵ 0��
	gp3.setMaximumNearestNeighbors(100);    //��������������ڵ���������
	gp3.setMaximumSurfaceAngle(M_PI / 4);      // 45 degrees��pi�����ƽ���
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees�������ڽ���С�Ƕ�
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees�������ڽ����Ƕ�
	gp3.setNormalConsistency(false);   //���øò�����֤���߳���һ�£����������һ�£�����Ϊtrue

	// Get result
	//���������������������
	gp3.setInputCloud(cloud_with_normals);  //�����������Ϊ�������cloud_with_normals
	gp3.setSearchMethod(tree2);  //����������ʽΪtree2
	gp3.reconstruct(triangles);	//ִ���ع������������triangles��
	
	//��������ͼ
	pcl::io::savePLYFile("result.ply", triangles);
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	// Finish����

	//���ӻ�
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0); //���ñ���
	viewer->addPolygonMesh(triangles, "my"); //������ʾ������
	//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1,0); //��������ϵ
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{

		viewer->spinOnce(100);

		boost::this_thread::sleep(boost::posix_time::microseconds(100000));

	}
    return (0);
}