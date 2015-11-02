#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;
std::string cloud = "cloud";
std::string partial_cloud = "partial_cloud";


void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

int 
loadXYZFile(const std::string &file_name, pcl::PointCloud< PointT > &cloud)
{

  std::ifstream file(file_name);
  std::string str;
  std::string file_contents;
  std::vector< std::vector<double> > pointCloud;

  while (std::getline(file, str))
  {
    std::istringstream iss(str);
    std::string xyz;
    std::vector<double> point;
    while(std::getline(iss,xyz,' '))

    {
      double temp = atof(xyz.c_str());
      point.push_back(temp);
    }
      
    pointCloud.push_back(point);
    point.clear();
  }

  cloud.width    = pointCloud.size();
  std::cout << cloud.width;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = pointCloud[i][0];
    cloud.points[i].y = pointCloud[i][1];
    cloud.points[i].z = pointCloud[i][2];
  }

  std::cout << std::endl << "String file_name: " << file_name.substr(0,file_name.size()) << std::endl;

  pcl::io::savePCDFileASCII (file_name.substr(0,file_name.size() - 4) + ".pcd", cloud);

  return 1;

}
int
main (int argc, char* argv[])
{
  // The point clouds we will be using
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  PointCloudT::Ptr cloud_filtered_outliers (new PointCloudT);

  pcl::console::TicToc time;

  // Checking program arguments
  if (argc < 2)
  {
    printf ("Usage :\n");
    printf ("\t\t%s number_of_clouds number_of_ICP_iterations_per_cloud\n", argv[0]);
    PCL_ERROR ("Provide number_of_clouds.\n");
    return (-1);
  }

  if (atoi (argv[1]) < 2)
  {
    PCL_ERROR ("number of clouds must be >= 2\n");
    return (-1);
  }

  int iterations = 1;  // Default number of ICP iterations
  if (argc > 2)
  {
    // If the user passed the number of iteration as an argument
    iterations = atoi (argv[2]);
    if (iterations < 1)
    {
      PCL_ERROR ("Number of initial iterations must be >= 1\n");
      return (-1);
    }
  }


  

  //Load first cloud (reference cloud)
  
  std::stringstream sstm;
  sstm << cloud << 1 << ".xyz";
  std::string file_name = sstm.str();

  time.tic ();
  if (loadXYZFile (file_name, *cloud_icp) < 0)
  {
    PCL_ERROR ("Error loading cloud %n.\n", 1);
    return (-1);
  }
  std::cout << "\nLoaded file " << file_name << " (" << cloud_icp->size () << " points) in " << time.toc () << " ms\n" << std::endl;    

  


  //APPLY STATISTICAL OUTLIER REMOVAL
  std::cout << "Applying StatisticalOutlierRemoval" << std::endl;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
  sor1.setInputCloud (cloud_icp);
  sor1.setMeanK (50);
  sor1.setStddevMulThresh (1.0);
  sor1.filter (*cloud_filtered_outliers);



  //APPLY VOXEL GRID FILTER TO DOWNSAMPLE THE CLOUD
  std::cout << "Applying VoxelGrid filter" << std::endl;
  pcl::VoxelGrid<PointT> vgf1;
  vgf1.setInputCloud (cloud_filtered_outliers);
  vgf1.setLeafSize (0.1f, 0.1f, 0.1f);
  vgf1.filter (*cloud_filtered);


  *cloud_icp = *cloud_filtered;

  //SAVE FILE
  std::stringstream sst;
  sst << partial_cloud << 1 << ".xyz";
  file_name = sst.str();
  pcl::io::savePCDFileASCII (file_name, *cloud_icp);

  //START PROCESS OF CONCATENATING CLOUDS ITERATIVELY
  for(int i = 2; i <= atoi (argv[1]); i++)
  {


    //Load cloud 
    std::stringstream ss;
    ss << cloud << i << ".xyz";
    file_name = ss.str();

    time.tic ();
    if (loadXYZFile (file_name, *cloud_in) < 0)
    {
      PCL_ERROR ("Error loading cloud %n.\n", i);
      return (-1);
    }
    std::cout << "\nLoaded file " << file_name << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;



    //APPLY STATISTICAL OUTLIER REMOVAL
    std::cout << "Applying StatisticalOutlierRemoval" << std::endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud (cloud_in);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (1.0);
    sor2.filter (*cloud_filtered_outliers);


    //APPLY VOXEL GRID FILTER TO DOWNSAMPLE THE CLOUD
    std::cout << "Applying VoxelGrid filter" << std::endl;
    pcl::VoxelGrid<PointT> vgf2;
    vgf2.setInputCloud (cloud_filtered_outliers);
    vgf2.setLeafSize (0.1f, 0.1f, 0.1f);
    vgf2.filter (*cloud_filtered);

    *cloud_in = *cloud_filtered;


    // The Iterative Closest Point algorithm
    time.tic ();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (cloud_in);
    icp.align (*cloud_icp);
    // icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;


    if (icp.hasConverged ())
    {
      std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
      std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
      Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
      transformation_matrix = icp.getFinalTransformation ().cast<double>();
      print4x4Matrix (transformation_matrix);
    }
    else
    {
      PCL_ERROR ("\nICP has not converged.\n");
      return (-1);
    }

    
    


    std::cout << "Concatenating Clouds" << std::endl;
    *cloud_icp+= *cloud_in;


    //APPLY STATISTICAL OUTLIER REMOVAL

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_icp);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered_outliers);



    //APPLY VOXEL GRID FILTER TO DOWNSAMPLE THE CLOUD

    std::cout << "Applying VoxelGrid filter" << std::endl;
    pcl::VoxelGrid<PointT> vgf;
    vgf.setInputCloud (cloud_filtered_outliers);
    vgf.setLeafSize (0.1f, 0.1f, 0.1f);
    vgf.filter (*cloud_filtered);



    //LAST STEP
    std::stringstream sstm;
    sstm << partial_cloud  << i << ".xyz";
    file_name = sstm.str();

    pcl::io::savePCDFileASCII (file_name, *cloud_filtered);

    *cloud_icp = *cloud_filtered;







  }

  return (0);
}