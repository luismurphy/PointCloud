#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);//edgeCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);//halfEdgeCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_outliers (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    
  //Load file1
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud1.pcd", *cloud_in) == -1) //* load the file
  {
      PCL_ERROR ("Couldn't read file cloud1.pcd \n");
      return (-1);
  }
  std::cout << "Loaded " << cloud_in->width * cloud_in->height << " data points from cloud1.pcd " << std::endl;


  //APPLY STATISTICAL OUTLIER REMOVAL
  std::cout << "Applying StatisticalOutlierRemoval" << std::endl;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
  sor1.setInputCloud (cloud_in);
  sor1.setMeanK (50);
  sor1.setStddevMulThresh (1.0);
  sor1.filter (*cloud_filtered_outliers);


  //APPLY VOXEL GRID FILTER TO DOWNSAMPLE THE CLOUD
  std::cout << "Applying VoxelGrid filter" << std::endl;
  pcl::VoxelGrid<PointT> vgf1;
  vgf1.setInputCloud (cloud_filtered_outliers);
  vgf1.setLeafSize (0.1f, 0.1f, 0.1f);
  vgf1.filter (*cloud_filtered);

  *cloud_in = *cloud_filtered;


  pcl::io::savePCDFileASCII ("cloud1.xyz", *cloud_in);
  std::cerr << "Saved " << cloud_in->points.size () << " data points to cloud1.xyz" << std::endl;
  
  //Load file2
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud2.pcd", *cloud_out) == -1) //* load the file
  {
      PCL_ERROR ("Couldn't read file cloud2.pcd \n");
      return (-1);
  }
  std::cout << "Loaded " << cloud_out->width * cloud_out->height << " data points from cloud2.pcd " << std::endl;


  //APPLY STATISTICAL OUTLIER REMOVAL
  std::cout << "Applying StatisticalOutlierRemoval" << std::endl;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud (cloud_out);
  sor2.setMeanK (50);
  sor2.setStddevMulThresh (1.0);
  sor2.filter (*cloud_filtered_outliers);


  //APPLY VOXEL GRID FILTER TO DOWNSAMPLE THE CLOUD
  std::cout << "Applying VoxelGrid filter" << std::endl;
  pcl::VoxelGrid<PointT> vgf2;
  vgf2.setInputCloud (cloud_filtered_outliers);
  vgf2.setLeafSize (0.1f, 0.1f, 0.1f);
  vgf2.filter (*cloud_filtered);

  *cloud_out = *cloud_filtered;

  pcl::io::savePCDFileASCII ("cloud2.xyz", *cloud_out);
  std::cerr << "Saved " << cloud_out->points.size () << " data points to cloud2.xyz" << std::endl;

  
  pcl::PointCloud<pcl::PointXYZ> stitchedCloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  icp.setMaximumIterations(500);
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
    
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  //icp.setMaxCorrespondenceDistance (100);
  // Set the maximum number of iterations (criterion 1)
  //icp.setMaximumIterations (50);
  // Set the transformation epsilon (criterion 2)
  //icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  //icp.setEuclideanFitnessEpsilon (1e-5);

  //From PCL DOCS
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  //icp.setMaxCorrespondenceDistance (0.05);
  // Set the maximum number of iterations (criterion 1)
  //icp.setMaximumIterations (50);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  //icp.setEuclideanFitnessEpsilon (1);

    
  //icp.setMaxCorrespondenceDistance(100);
  //icp.setMaximumIterations(10);
  //icp.setTransformationEpsilon(1e-2);
  //icp.setEuclideanFitnessEpsilon(1e-5);
    
  icp.align(*transformedCloud);  
  //Get Parameters
  std::cout << std::endl;
  std::cout << "Max number of Iterations: " << icp.getMaximumIterations() << std::endl;
  std::cout << "Eucledian Fitness Epsilon: " << icp.getEuclideanFitnessEpsilon() << std::endl;
  std::cout << "Transformation Epsilon: " << icp.getTransformationEpsilon() << std::endl;
  std::cout << "Max Correspondance Distance :" << icp.getMaxCorrespondenceDistance() << std::endl;
  std::cout << std::endl;

  pcl::io::savePCDFileASCII ("transformed_cloud.pcd", *transformedCloud);
  pcl::io::savePCDFileASCII ("transformed_cloud.xyz", *transformedCloud);
  std::cerr << "Saved " << transformedCloud->points.size () << " data points to transformed_cloud.pcd." << std::endl;

  //APPLY STATISTICAL OUTLIER REMOVAL
  std::cout << "Applying StatisticalOutlierRemoval" << std::endl;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor3;
  sor3.setInputCloud (transformedCloud);
  sor3.setMeanK (50);
  sor3.setStddevMulThresh (1.0);
  sor3.filter (*cloud_filtered_outliers);


  //APPLY VOXEL GRID FILTER TO DOWNSAMPLE THE CLOUD
  std::cout << "Applying VoxelGrid filter" << std::endl;
  pcl::VoxelGrid<PointT> vgf3;
  vgf3.setInputCloud (cloud_filtered_outliers);
  vgf3.setLeafSize (0.1f, 0.1f, 0.1f);
  vgf3.filter (*cloud_filtered);

  pcl::io::savePCDFileASCII ("transformed_cloud_filtered.pcd", *cloud_filtered);
  pcl::io::savePCDFileASCII ("transformed_cloud_filtered.xyz", *cloud_filtered);

    
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

    
  //Concatenate point clouds
  //stitchedCloud = *cloud_out;
  //stitchedCloud+= transformedCloud;
    
    
  //pcl::io::savePCDFileASCII ("stitched_cloud.pcd", stitchedCloud);
  //pcl::io::savePCDFileASCII ("stitched_cloud.xyz", stitchedCloud);
  //std::cerr << "Saved " << stitchedCloud.points.size () << " data points to stitched_cloud.pcd." << std::endl;

    

 return (0);
}