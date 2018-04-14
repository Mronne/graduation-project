#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

using namespace std;

//-------------------------------------------------------//
//                   FeatureCloud类                      //
//主要功能：提供一个计算和存储点云中每点的局部特征描述算子方法      //
//【1】构造函数 FeatureCloud()  创建一个KDTreeFLANN对象并初始化半径 //
//						  变量，从而用于计算表面法线和局部特征  //
//【2】setInputCloud()定义处理输入点云的方法                  //
//【3】loadInputCloud()定义处理PCD文件的方法                 //
//【4】getPointCloud()获取指向点云函数                       //
//【5】getSurfaceNormals()获取表面法相函数                   //
//【6】getLocalFeatures()获取局部特征描述符函数               //
//【7】processInput() 定义处理输入的点云的方法                //
//【8】computeSurfaceNormals();计算点云的表面法线            //
//【9】computeLocalFeatures();计算点云局部特征              //  
class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
	//利用PCL的NormalEstimation类来计算表面法线---------------------//
	//1.指定输入点云
    //2.KdTree用来搜索相邻点，“半径”定义了各点的邻域
	//3.计算表面法线并存储于成员变量中
	void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
	//通过输入点云及其表面法线来计算“快速点特征直方图”（“Fast Point Feature Histogram”）描述算子。 
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};
//-------------------------------------------------------//
//-------------------------------------------------------//


//-------------------------------------------------------//
//                TemplateAlignment类                    //
//此类主要用于模板对齐。模板通常是指一组规模较小的像素点或三维点，   //
//来源于更大的已知物体或场景。通过将模板与新获得的图像或点云进行匹配，//
//就可以获得模板所代表物体的位置及方向。                        //

/*【1】结构体：存储对齐结果
   包括一个表示匹配度(fitness)的浮点数(值越低匹配效果越好)
   旋转矩阵，描述模板上的点通过怎样的旋转和平移才能尽可能好地与目标点云匹配*/

/*【2】构造函数
初始化SampleConsensusInitialAlignment(SAC-IA)对象用来进行对齐操作
为每个参量提供了初始值。
最大通信距离(correspondence distance)实际上被指定为距离值的平方
我们决定限制误差上限为1厘米，所以实际传递的值为0.01平方厘米。*/

/*【3】setTargetCloud()设定目标点云(即与模板对齐的点云)
	按照 SAC-IA 对齐方法设置输入*/

/*【4】addTemplateCloud() 指定哪个或哪些模板被用来匹配
每次调用这个方法都会把给定的模板添加到 FeatureClouds 的一个
内部向量中存储起来，以便后续使用。*/

/*【5】align()  定义对齐方法
这一方法将模板作为输入并将其与通过调用 setInputTarget() 方法指定的目标点进行匹配。
其工作过程为：
1.设置给定模板为 SAC-IA 算法的源点云(source cloud)
2.调用匹配算法将源与目标进行匹配。
！！注意：这一匹配算法需要我们传入一个点云用来存储新对齐的点云，但是我们的应用可以忽略这个输出。
作为代替，我们调用SAC-IA的存取器方法来获得对齐的匹配度以及最终的变换矩阵，并且以 Result 结构体的形式输出。 */

//【6】alignAll()定义Result型向量用于存储所有模板的对齐结果   //

/*【7】findBestAlignment()
	   将所有模板与点云对齐
	   并返回最佳对齐的序号及其Result结构体*/
class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (500)
    {
      // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

// Align a collection of object templates to a sample point cloud
int
main (int argc, char **argv)
{
  // Load the object templates specified in the object_templates.txt file
  // 读入每个文件名，加载到 FeatureCloud 中，并存入向量。
  std::vector<FeatureCloud> object_templates;
  std::ifstream input_stream ("object_templates.txt");
  object_templates.resize (0);
  std::string pcd_filename;
  while (input_stream.good ())
  {
    std::getline (input_stream, pcd_filename);
    if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
      continue;

    FeatureCloud template_cloud;
    template_cloud.loadInputCloud (pcd_filename);
    object_templates.push_back (template_cloud);
  }
  input_stream.close ();
  cout<<"读取点云成功"<<endl;


  // Load the target cloud PCD file
  // 加载目标点云文件
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile ("object_template_0.pcd", *cloud);
  cout<<"加载目标点云成功"<<endl;

  //--------------------点云预处理-----------------------------//
  //【1】依据深度过滤背景点
  //使用通道滤波，保留Z向（即深度方向）0至1米的范围。 
  const float depth_limit = 1.0;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, depth_limit);
  pass.filter (*cloud);
  cout<<"处理成功"<<endl;
  //【2】点云降采样到点距5mm，降低所需计算量
  const float voxel_grid_size = 0.005f;
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setInputCloud (cloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  cout<<1<<endl;
  //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
  //【3】生成目标的FeatureCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
  vox_grid.filter (*tempCloud);
  cloud = tempCloud; 
  cout<<2<<endl;
  // 【4】 初始化TemplateAlignment物体
  //7要加入所有模板点云并设定目标点云
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (cloud);
  cout<<3<<endl;
  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < object_templates.size (); ++i)
  {
    template_align.addTemplateCloud (object_templates[i]);
  }
  cout<<4<<endl;
  template_align.setTargetCloud (target_cloud);
  cout<<5<<endl;
  // 【5】确定哪个模板与目标点云配合的最好
  //将对齐结果保存到 best_alignment 。
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment (best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
  Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

  // Save the aligned template for visualization
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
  pcl::io::savePCDFileBinary ("person.pcd", transformed_cloud);

  return (0);
}