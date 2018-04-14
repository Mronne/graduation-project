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
//                   FeatureCloud��                      //
//��Ҫ���ܣ��ṩһ������ʹ洢������ÿ��ľֲ������������ӷ���      //
//��1�����캯�� FeatureCloud()  ����һ��KDTreeFLANN���󲢳�ʼ���뾶 //
//						  �������Ӷ����ڼ�����淨�ߺ;ֲ�����  //
//��2��setInputCloud()���崦��������Ƶķ���                  //
//��3��loadInputCloud()���崦��PCD�ļ��ķ���                 //
//��4��getPointCloud()��ȡָ����ƺ���                       //
//��5��getSurfaceNormals()��ȡ���淨�ຯ��                   //
//��6��getLocalFeatures()��ȡ�ֲ���������������               //
//��7��processInput() ���崦������ĵ��Ƶķ���                //
//��8��computeSurfaceNormals();������Ƶı��淨��            //
//��9��computeLocalFeatures();������ƾֲ�����              //  
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
	//����PCL��NormalEstimation����������淨��---------------------//
	//1.ָ���������
    //2.KdTree�����������ڵ㣬���뾶�������˸��������
	//3.������淨�߲��洢�ڳ�Ա������
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
	//ͨ��������Ƽ�����淨�������㡰���ٵ�����ֱ��ͼ������Fast Point Feature Histogram�����������ӡ� 
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
//                TemplateAlignment��                    //
//������Ҫ����ģ����롣ģ��ͨ����ָһ���ģ��С�����ص����ά�㣬   //
//��Դ�ڸ������֪����򳡾���ͨ����ģ�����»�õ�ͼ�����ƽ���ƥ�䣬//
//�Ϳ��Ի��ģ�������������λ�ü�����                        //

/*��1���ṹ�壺�洢������
   ����һ����ʾƥ���(fitness)�ĸ�����(ֵԽ��ƥ��Ч��Խ��)
   ��ת��������ģ���ϵĵ�ͨ����������ת��ƽ�Ʋ��ܾ����ܺõ���Ŀ�����ƥ��*/

/*��2�����캯��
��ʼ��SampleConsensusInitialAlignment(SAC-IA)�����������ж������
Ϊÿ�������ṩ�˳�ʼֵ��
���ͨ�ž���(correspondence distance)ʵ���ϱ�ָ��Ϊ����ֵ��ƽ��
���Ǿ��������������Ϊ1���ף�����ʵ�ʴ��ݵ�ֵΪ0.01ƽ�����ס�*/

/*��3��setTargetCloud()�趨Ŀ�����(����ģ�����ĵ���)
	���� SAC-IA ���뷽����������*/

/*��4��addTemplateCloud() ָ���ĸ�����Щģ�屻����ƥ��
ÿ�ε��������������Ѹ�����ģ����ӵ� FeatureClouds ��һ��
�ڲ������д洢�������Ա����ʹ�á�*/

/*��5��align()  ������뷽��
��һ������ģ����Ϊ���벢������ͨ������ setInputTarget() ����ָ����Ŀ������ƥ�䡣
�乤������Ϊ��
1.���ø���ģ��Ϊ SAC-IA �㷨��Դ����(source cloud)
2.����ƥ���㷨��Դ��Ŀ�����ƥ�䡣
����ע�⣺��һƥ���㷨��Ҫ���Ǵ���һ�����������洢�¶���ĵ��ƣ��������ǵ�Ӧ�ÿ��Ժ�����������
��Ϊ���棬���ǵ���SAC-IA�Ĵ�ȡ����������ö����ƥ����Լ����յı任���󣬲����� Result �ṹ�����ʽ����� */

//��6��alignAll()����Result���������ڴ洢����ģ��Ķ�����   //

/*��7��findBestAlignment()
	   ������ģ������ƶ���
	   ��������Ѷ������ż���Result�ṹ��*/
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
  // ����ÿ���ļ��������ص� FeatureCloud �У�������������
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
  cout<<"��ȡ���Ƴɹ�"<<endl;


  // Load the target cloud PCD file
  // ����Ŀ������ļ�
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile ("object_template_0.pcd", *cloud);
  cout<<"����Ŀ����Ƴɹ�"<<endl;

  //--------------------����Ԥ����-----------------------------//
  //��1��������ȹ��˱�����
  //ʹ��ͨ���˲�������Z�򣨼���ȷ���0��1�׵ķ�Χ�� 
  const float depth_limit = 1.0;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, depth_limit);
  pass.filter (*cloud);
  cout<<"����ɹ�"<<endl;
  //��2�����ƽ����������5mm���������������
  const float voxel_grid_size = 0.005f;
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setInputCloud (cloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  cout<<1<<endl;
  //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
  //��3������Ŀ���FeatureCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
  vox_grid.filter (*tempCloud);
  cloud = tempCloud; 
  cout<<2<<endl;
  // ��4�� ��ʼ��TemplateAlignment����
  //7Ҫ��������ģ����Ʋ��趨Ŀ�����
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
  // ��5��ȷ���ĸ�ģ����Ŀ�������ϵ����
  //�����������浽 best_alignment ��
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