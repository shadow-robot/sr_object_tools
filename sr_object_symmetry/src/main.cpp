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
int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_symmetry");
  sym::RotSymDetectParams rotDetParams;
  sym::ReflSymDetectParams reflDetParams;
  int objectSampling = 15000;
  bool visEnable, overWrite = true;
  std::string yamlPath, objectsPath, fileName, yamlFile;
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
      ROS_INFO("Folder scanning mode");
      dir = opendir(objectsPath.c_str());
      while ((dirp = readdir(dir)) != NULL)
      {
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
    // Create new Symmetries object
    SymmetryDetection symmetries_T;
    reflDetParams.rot_symmetries.clear();
    yamlFile = yamlPath + "/" + utl::getBasenameNoExtension(files[fileIndex].c_str()) + ".yaml";
    // First check if file exists, if not then extract symmetries
    if ((utl::isFile(yamlFile) == false) || (overWrite == true))
    {
      ROS_INFO_STREAM("Loading object: " << files[fileIndex].c_str());
      symmetries_T.loadFile(files[fileIndex].c_str(), objectSampling);
      // Detect rotational symmetries
      symmetries_T.rotationalDetection<PointT>(rotDetParams);
      if (symmetries_T.getRotational().size() == 0)
      {
        ROS_WARN("Could not find rotational symmetries");
      }
      else
      {
        ROS_INFO_STREAM("Rotational symmetries: " << symmetries_T.getRotational().size());
        for (size_t symId = 0; symId < symmetries_T.getRotational().size(); symId++)
          reflDetParams.rot_symmetries.push_back(symmetries_T.getRotational()[symId].getDirection());
      }
      // Detect reflectional symmetries
      symmetries_T.reflectionalDetection<PointT>(reflDetParams);
      if (symmetries_T.getReflectional().size() == 0)
        ROS_WARN("Could not find reflectional symmetries");
      else
      {
        ROS_INFO_STREAM("Reflectional symmetries: " << symmetries_T.getReflectional().size());
      }
      if (visEnable)
      {
        // For visualization
        pcl::PointCloud<PointT>::Ptr cloudHighRes(new pcl::PointCloud<PointT>);
        cloudHighRes = symmetries_T.getCloud();
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
              visualizer.addText("Numpad 0: Delete current symmetry", 0, 0, 24, 1.0, 1.0, 1.0);
              visualizer.addText("Numpad 1: Show Point Cloud", 0, 20, 24, 1.0, 1.0, 1.0);
              visualizer.addText("Numpad 3: Show Rotational Symmetries", 0, 40, 24, 1.0, 1.0, 1.0);
              visualizer.addText("Numpad 7: Show Reflectional Symmetries", 0, 60, 24, 1.0, 1.0, 1.0);
              visualizer.addText("Q: Go to next object", 0, 80, 24, 1.0, 1.0, 1.0);
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
              if (visState.delete_)
              {
                visState.delete_ = false;
                symmetries_T.removeCurrRotational(symId);
                if (symmetries_T.getRotational().size() == 0)
                  visState.cloudDisplay_ = VisState::CLOUD;
                visState.updateDisplay_ = true;
                visState.showSymmetry_ = false;
              }
              // Show symmetry
              if (symmetries_T.getRotational().size() > 0)
              {
                if (visState.showSymmetry_)
                  sym::showRotationalSymmetry(visualizer, symmetryDisplay[symId], "symmetry", 5, 5.0);

                text = "Rotational symmetries score:" + std::to_string(symmetries_T.getRotationalScores()[symId]);
                visualizer.addText(text, 0, 150, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Numpad 0: Delete current symmetry", 0, 0, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Numpad 1: Show Point Cloud", 0, 20, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Numpad 3: Show Rotational Symmetries", 0, 40, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Numpad 7: Show Reflectional Symmetries", 0, 60, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Q: Go to next object", 0, 80, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Symmetry index ID " + std::to_string(visState.segIterator_) + " Size " +
                                       std::to_string(symmetryDisplayIds.size()),
                                   15, 125, 24, 1.0, 1.0, 1.0);
              }
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
              
              utl::showPointCloudColor<PointT>(visualizer, cloudHighRes, "cloud", visState.pointSize_);
              if (visState.delete_)
              {
                visState.delete_ = false;
                symmetries_T.removeCurrReflectional(symId);
                if (symmetries_T.getReflectional().size() == 0)
                  visState.cloudDisplay_ = VisState::CLOUD;
                visState.updateDisplay_ = true;
              }
              // Show symmetry
              if (symmetries_T.getReflectional().size() > 0)
              {
                if (visState.showSymmetry_)
                  sym::showReflectionalSymmetry(visualizer, symmetryDisplay[symId], "symmetry", 1);
                text = "cloudInlierScores Score: " + std::to_string(symmetries_T.getReflectionalScores()[symId]);
                visualizer.addText("Numpad 0: Delete current symmetry", 0, 0, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Numpad 1: Show Point Cloud", 0, 20, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Numpad 3: Show Rotational Symmetries", 0, 40, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Numpad 7: Show Reflectional Symmetries", 0, 60, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Q: Go to next object", 0, 80, 24, 1.0, 1.0, 1.0);
                visualizer.addText(text, 0, 100, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Symmetry index ID" + std::to_string(visState.segIterator_) + " Size " +
                                       std::to_string(symmetryDisplayIds.size()),
                                   15, 125, 24, 1.0, 1.0, 1.0);
              }
            }
          }
          // Spin once
          visualizer.spinOnce();
          boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        }
      }
      YAML::Emitter yamlOut;
      yamlOut << YAML::BeginMap;
      if (symmetries_T.getRotational().size() > 0)
      {
        yamlOut << YAML::Key << "rotational" << YAML::Value << YAML::BeginSeq;
        for (size_t symId = 0; symId < symmetries_T.getRotational().size(); symId++)
        {
          yamlOut << YAML::BeginMap << YAML::Key << "origin" << YAML::Value << YAML::Flow <<
                     YAML::BeginSeq << symmetries_T.getRotational()[symId].getOrigin()[0] <<
                     symmetries_T.getRotational()[symId].getOrigin()[1] <<
                     symmetries_T.getRotational()[symId].getOrigin()[2] << YAML::EndSeq;
          yamlOut << YAML::Key << "direction" << YAML::Value << YAML::Flow << YAML::BeginSeq <<
                     symmetries_T.getRotational()[symId].getDirection()[0] <<
                     symmetries_T.getRotational()[symId].getDirection()[1] <<
                     symmetries_T.getRotational()[symId].getDirection()[2] << YAML::EndSeq << YAML::EndMap;
        }
        yamlOut << YAML::EndSeq;
      }
      if (symmetries_T.getReflectional().size() > 0)
      {
        yamlOut << YAML::Key << "reflectional" << YAML::Value << YAML::BeginSeq;
        for (size_t symId = 0; symId < symmetries_T.getReflectional().size(); symId++)
        {
          yamlOut << YAML::BeginMap << YAML::Key << "origin" << YAML::Value << YAML::Flow <<
                     YAML::BeginSeq << symmetries_T.getReflectional()[symId].getOrigin()[0] <<
                     symmetries_T.getReflectional()[symId].getOrigin()[1] <<
                     symmetries_T.getReflectional()[symId].getOrigin()[2] << YAML::EndSeq;
          yamlOut << YAML::Key << "normal" << YAML::Value << YAML::Flow << YAML::BeginSeq <<
                     symmetries_T.getReflectional()[symId].getNormal()[0] <<
                     symmetries_T.getReflectional()[symId].getNormal()[1] <<
                     symmetries_T.getReflectional()[symId].getNormal()[2] << YAML::EndSeq << YAML::EndMap;
        }
        yamlOut << YAML::EndSeq;
      }
      yamlOut << YAML::EndMap;
      // Saving
      // First check if the output folder exists, if not then create it
      if ((symmetries_T.getReflectional().size() > 0) || (symmetries_T.getRotational().size() > 0))
      {
        if (!utl::isDirectory(yamlPath))
          utl::createDir(yamlPath);
        std::ofstream fout(yamlFile);
        fout << yamlOut.c_str();
        fout.close();
        ROS_INFO_STREAM("Yaml saved as:" << yamlFile);
      }
    }
  }
}
