/**
 * Copyright 2017 Aleksandrs Ecins
 * Copyright (C) 2018 Shadow Robot Company Ltd
￼ * Licensed under GPLv2+
 * Refer to the LICENSE.txt file included
**/

#include <pcl/pcl_config.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <dirent.h>
#include "yaml-cpp/yaml.h"
#include <string>
#include <vector>
#include <fstream>
#include "sr_object_symmetry/symmetry_detection.hpp"
#include "utilities/eigen.hpp"
#include "utilities/filesystem/filesystem.hpp"
#include "utilities/pointcloud/mesh_to_cloud.hpp"
#include "utilities/visualization/vis.hpp"
#include <sr_object_symmetry/reflectional_symmetry_detection.hpp>
#include <sr_object_symmetry/rotational_symmetry_detection.hpp>
#include <utilities/pointcloud/pointcloud.hpp>

typedef pcl::PointXYZRGBNormal PointT;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_symmetry");

  sym::RotSymDetectParams rotDetParams;
  sym::ReflSymDetectParams reflDetParams;
  SymmetryDetection symmetries_T;
  int objectSampling = 15000;
  bool visEnable, overWrite = true;
  std::string yamlPath, objectsPath, fileName;
  std::vector<std::string> files;
  DIR *dir;
  // Rotatioanal symmetry detection parameters
  ros::param::get("~ref_max_fit_angle", rotDetParams.ref_max_fit_angle);
  rotDetParams.ref_max_fit_angle = pcl::deg2rad(rotDetParams.ref_max_fit_angle);
  ros::param::get("~min_normal_fit_angle", rotDetParams.min_normal_fit_angle);
  rotDetParams.min_normal_fit_angle = pcl::deg2rad(rotDetParams.min_normal_fit_angle);
  ros::param::get("~max_normal_fit_angle", rotDetParams.max_normal_fit_angle);
  rotDetParams.max_normal_fit_angle = pcl::deg2rad(rotDetParams.max_normal_fit_angle);
  ros::param::get("~max_symmetry_score", rotDetParams.max_symmetry_score);
  ros::param::get("~max_perpendicular_score", rotDetParams.max_perpendicular_score);
  ros::param::get("~min_coverage_score", rotDetParams.min_coverage_score);

  // Reflectional symmetry detection parameters
  ros::param::get("~voxel_size", reflDetParams.voxel_size);
  ros::param::get("~num_angle_divisions", reflDetParams.num_angle_divisions);
  ros::param::get("~flatness_threshold", reflDetParams.flatness_threshold);
  ros::param::get("~refine_iterations", reflDetParams.refine_iterations);
  ros::param::get("~max_correspondence_reflected_distance", reflDetParams.max_correspondence_reflected_distance);
  ros::param::get("~min_inlier_normal_angle", reflDetParams.min_inlier_normal_angle);
  reflDetParams.min_inlier_normal_angle = pcl::deg2rad(reflDetParams.min_inlier_normal_angle);
  ros::param::get("~max_inlier_normal_angle", reflDetParams.max_inlier_normal_angle);
  reflDetParams.max_inlier_normal_angle = pcl::deg2rad(reflDetParams.max_inlier_normal_angle);
  ros::param::get("~min_cloud_inlier_score", reflDetParams.min_cloud_inlier_score);
  ros::param::get("~min_corresp_inlier_score", reflDetParams.min_corresp_inlier_score);
  ros::param::get("~symmetry_min_angle_diff", reflDetParams.symmetry_min_angle_diff);
  reflDetParams.symmetry_min_angle_diff = pcl::deg2rad(reflDetParams.symmetry_min_angle_diff);
  ros::param::get("~symmetry_min_distance_diff", reflDetParams.symmetry_min_distance_diff);
  ros::param::get("~max_reference_point_distance", reflDetParams.max_reference_point_distance);
  // Generic patameters
  ros::param::get("~visualization", visEnable);
  ros::param::get("~overwrite", overWrite);
  ros::param::get("~yaml_path", yamlPath);
  ros::param::get("~objects_path", objectsPath);
  ros::param::get("~object_sampling", objectSampling);
  ROS_INFO("Parameters loaded");
  // Check if objects_path is a single file or a folder
  struct stat s;
  struct dirent *dirp;
  if (stat(objectsPath.c_str(), &s) == 0)
  {
    if (s.st_mode & S_IFDIR)
    {
      ROS_INFO("Folder scanning");
      dir = opendir(objectsPath.c_str());
      while ((dirp = readdir(dir)) != NULL) {
        std::string fname = dirp->d_name;
        if (fname.find(".ply") != std::string::npos)
          files.push_back(objectsPath + "/" + fname);
      }
      closedir(dir);
    }
    else if (s.st_mode & S_IFREG)
    {
      files.push_back(objectsPath);
      ROS_INFO("Single file defined");
    }
  }
  else
  {
    ROS_INFO_STREAM("Error checking input path: " << objectsPath.c_str());
    return 0;
  }
  for (size_t fileIndex = 0; fileIndex < files.size(); fileIndex++)
  {
    YAML::Emitter yamlOut;
    // yamlOut << YAML::BeginMap;
    ROS_INFO_STREAM("Loading object: " << files[fileIndex].c_str());
    symmetries_T.loadFile(files[fileIndex].c_str(), objectSampling);
    // For visualization
    pcl::PointCloud<PointT>::Ptr cloudHighRes(new pcl::PointCloud<PointT>);
    cloudHighRes = symmetries_T.getCloud();
    // Detect rotational symmetries
    symmetries_T.rotationalDetection<PointT>(rotDetParams);
    if (symmetries_T.getRotational().size() == 0)
    {
      ROS_WARN("Could not find rotational symmetries");
    }
    else
    {
      yamlOut << YAML::BeginMap << YAML::Key << "rotational" << YAML::Value << YAML::BeginSeq;
      ROS_INFO_STREAM("Rotational symmetries: " << symmetries_T.getRotational().size());
      for (size_t symId = 0; symId < symmetries_T.getRotational().size(); symId++)
      {
        yamlOut << YAML::BeginMap << YAML::Key << "origin" << YAML::Value << YAML::Flow << YAML::BeginSeq << symmetries_T.getRotational()[symId].getOrigin()[0] << symmetries_T.getRotational()[symId].getOrigin()[1] << symmetries_T.getRotational()[symId].getOrigin()[2] << YAML::EndSeq;
        yamlOut << YAML::Key << "direction" << YAML::Value << YAML::Flow << YAML::BeginSeq << symmetries_T.getRotational()[symId].getDirection()[0] << symmetries_T.getRotational()[symId].getDirection()[1] << symmetries_T.getRotational()[symId].getDirection()[2] << YAML::EndSeq << YAML::EndMap;
        
        reflDetParams.rot_symmetries.push_back(symmetries_T.getRotational()[symId].getDirection());
        ROS_INFO_STREAM("Rotational symmetry ORIGIN ID:" << symId << ":" << symmetries_T.getRotational()[symId].getOrigin());
        ROS_INFO_STREAM("Rotational symmetry DIRECTION ID:" << symId << ":" << symmetries_T.getRotational()[symId].getDirection());
      }
      yamlOut << YAML::EndSeq << YAML::EndMap;
    }

    // Detect reflectional symmetries
    symmetries_T.reflectionalDetection<PointT>(reflDetParams);
    if (symmetries_T.getReflectional().size() == 0)
      ROS_WARN("Could not find reflectional symmetries");
    else
    {
      yamlOut << YAML::BeginMap << YAML::Key << "reflectional" << YAML::Value << YAML::BeginSeq;
      ROS_INFO_STREAM("Reflectional symmetries: " << symmetries_T.getReflectional().size());
      for (size_t symId = 0; symId < symmetries_T.getReflectional().size(); symId++)
      {
        yamlOut << YAML::BeginMap << YAML::Key << "origin" << YAML::Value << YAML::Flow << YAML::BeginSeq << symmetries_T.getReflectional()[symId].getOrigin()[0] << symmetries_T.getReflectional()[symId].getOrigin()[1] << symmetries_T.getReflectional()[symId].getOrigin()[2] << YAML::EndSeq;
        yamlOut << YAML::Key << "direction" << YAML::Value << YAML::Flow << YAML::BeginSeq << symmetries_T.getReflectional()[symId].getNormal()[0] << symmetries_T.getReflectional()[symId].getNormal()[1] << symmetries_T.getReflectional()[symId].getNormal()[2] << YAML::EndSeq << YAML::EndMap;
        ROS_INFO_STREAM("Reflectional symmetry ID:" << symId << ":" << symmetries_T.getReflectional()[symId]);
      }
      yamlOut << YAML::EndSeq << YAML::EndMap;
    }
    // yamlOut << YAML::EndMap;
    // Saving
    // {
    std::ofstream fout("/tmp/file.yaml");
    cout << yamlOut.c_str();
    fout << yamlOut.c_str();
    fout.close();
    // }
    if (visEnable)
    {
      //  Visualization
      std::cout << "Controls:" << std::endl;
      std::cout << "Numpad 1: Show Point Cloud" << std::endl;
      std::cout << "Numpad 3: Show Rotational Symmetries" << std::endl;
      std::cout << "Numpad 7: Show Reflectional Symmetries" << std::endl;
      std::cout << "Left/Right arrows: Show next symmetry in list" << std::endl;

      VisState visState;
      pcl::visualization::PCLVisualizer visualizer;
      visualizer.setCameraPosition(0.0, 0.0, -1.0,  // camera position
                                   0.0, 0.0, 1.0,   // viewpoint
                                   0.0, -1.0, 0.0,  // normal
                                   0.0);            // viewport
      visualizer.setBackgroundColor(utl::bgColor.r, utl::bgColor.g, utl::bgColor.b);
      visualizer.registerKeyboardCallback(keyboard_callback, reinterpret_cast<void *>(&visState));
      visState.updateDisplay_ = true;

      while (!visualizer.wasStopped())
      {
        // Update display if needed
        if (visState.updateDisplay_)
        {
          // First remove everything
          visualizer.removeAllPointClouds();
          visualizer.removeAllShapes();
#if PCL_VERSION_COMPARE(<, 1, 8, 0)
          visualizer.removeCoordinateSystem();
#else
          visualizer.removeAllCoordinateSystems();
#endif
          visState.updateDisplay_ = false;
          if (visState.cloudDisplay_ == VisState::CLOUD)
          {
            pcl::PointCloud<PointT>::Ptr cloudDisplay(new pcl::PointCloud<PointT>);

            std::string text;
            if (visState.cloudDisplay_ == VisState::CLOUD)
            {
              cloudDisplay = cloudHighRes;
              text = "Original cloud";
            }
            utl::showPointCloudColor<PointT>(visualizer, cloudDisplay, "cloud", visState.pointSize_);
            if (visState.showNormals_)
              utl::showNormalCloud<PointT>(visualizer, cloudDisplay, 10, 0.02, "normals",
                                           visState.pointSize_, utl::green);
            visualizer.addText(text, 0, 150, 24, 1.0, 1.0, 1.0);
          }
          else if ((visState.cloudDisplay_ == VisState::ROTATIONAL_SYMMETRIES) &&
                   (symmetries_T.getRotational().size() > 0))
          {
            std::vector<sym::RotationalSymmetry> symmetryDisplay;
            std::vector<int> symmetryDisplayIds;
            std::string text;
            symmetryDisplay = symmetries_T.getRotational();
            for (size_t symId = 0; symId < symmetries_T.getRotational().size(); symId++)
            {
              symmetryDisplayIds.push_back(symId);
            }
            visState.segIterator_ = utl::clampValueCircular<int>(visState.segIterator_, 0,
                                                                 symmetryDisplayIds.size() - 1);
            int symId = symmetryDisplayIds[visState.segIterator_];
            utl::showPointCloudColor<PointT>(visualizer, cloudHighRes, "cloud", visState.pointSize_);
            //  Show symmetry
            if (visState.showSymmetry_)
              sym::showRotationalSymmetry(visualizer, symmetryDisplay[symId], "symmetry", 5, 5.0);
            text = "Rotational symmetries score:" + std::to_string(symmetries_T.getRotationalScores()[symId]);
            visualizer.addText(text, 0, 150, 24, 1.0, 1.0, 1.0);
            visualizer.addText("Symmetry " + std::to_string(visState.segIterator_) + " / " +
                                   std::to_string(symmetryDisplayIds.size() - 1),
                               15, 125, 24, 1.0, 1.0, 1.0);
          }
          else if ((visState.cloudDisplay_ == VisState::REFLECTIONAL_SYMMETRIES) &&
                   (symmetries_T.getReflectional().size() > 0))
          {
            std::vector<sym::ReflectionalSymmetry> symmetryDisplay;
            std::vector<int> symmetryDisplayIds;
            std::string text;
            if (visState.cloudDisplay_ == VisState::REFLECTIONAL_SYMMETRIES)
            {
              symmetryDisplay = symmetries_T.getReflectional();
              for (size_t symId = 0; symId < symmetries_T.getReflectional().size(); symId++)
                symmetryDisplayIds.push_back(symId);
              text = "Reflectional symmetry ";
            }
            visState.segIterator_ = utl::clampValueCircular<int>(visState.segIterator_, 0,
                                                                 symmetryDisplayIds.size() - 1);
            int symId = symmetryDisplayIds[visState.segIterator_];
            text = "cloudInlierScores Score: " + std::to_string(symmetries_T.getReflectionalScores()[symId]);
            utl::showPointCloudColor<PointT>(visualizer, cloudHighRes, "cloud", visState.pointSize_);
            // Show symmetry
            if (visState.showSymmetry_)
              sym::showReflectionalSymmetry(visualizer, symmetryDisplay[symId], "symmetry", 1);
            visualizer.addText(text, 0, 150, 24, 1.0, 1.0, 1.0);
            visualizer.addText("Symmetry " + std::to_string(visState.segIterator_) + " / " +
                                   std::to_string(symmetryDisplayIds.size() - 1),
                               15, 125, 24, 1.0, 1.0, 1.0);
          }
        }
        // Spin once
        visualizer.spinOnce();
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
      }
    }
  }
}
