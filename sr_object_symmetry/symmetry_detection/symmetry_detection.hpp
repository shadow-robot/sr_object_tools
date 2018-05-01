/*
 * Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
 * Unauthorized copying of the content in this file, via any medium is strictly prohibited.
 *
*/
/**
 * @file   symmetry_detection.hpp
 * @author Fotios Papadopoulos <fotios@shadowrobot.com>
 * @brief  Extracts rotational and reflectional symmetries from point clouds
**/
// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef SYMMETRY_DETECTION_HPP
#define SYMMETRY_DETECTION_HPP

#include "eigen.hpp"
#include <pointcloud/pointcloud.hpp>
#include <rotational_symmetry_detection.hpp>
#include <reflectional_symmetry_detection.hpp>
#include "pointcloud/mesh_to_cloud.hpp"

class SymmetryDetection
{
  private:
    std::vector<sym::RotationalSymmetry> rot_symmetries;
    std::vector<sym::ReflectionalSymmetry> ref_symmetries;
    typedef pcl::PointXYZRGBNormal PointT;
    pcl::PointCloud<PointT>::Ptr cloudHighRes{new pcl::PointCloud<PointT>};
  public:
    void loadFile(std::string filename, int sampling)
    {
        cloudHighRes = convertPlyToCloud(filename, sampling);
    }
    pcl::PointCloud<PointT>::Ptr getCloud(void)
    {
        return cloudHighRes;
    }

    template <typename PointT>
    void rotationalDetection(sym::RotSymDetectParams &rot_parameters)
    {
        std::vector<sym::RotationalSymmetry> symmetry_TMP;
        std::vector<sym::RotationalSymmetry> symmetries;
        std::vector<int> symmetryFilteredIds_TMP;
        sym::RotSymDetectParams rotDetParams;
        std::vector<sym::RotationalSymmetry> symmetry_linear;
        std::vector<Eigen::Vector3f> referencePoints_linear;
        rotDetParams = rot_parameters;
        sym::RotationalSymmetryDetection<PointT> rsd(rotDetParams);
        rsd.setInputCloud(cloudHighRes);
        rsd.detect();
        rsd.filter();
        rsd.getSymmetries(symmetry_TMP, symmetryFilteredIds_TMP);
        // Merge
        for (size_t symIdIt = 0; symIdIt < symmetryFilteredIds_TMP.size(); symIdIt++)
        {
            int symId = symmetryFilteredIds_TMP[symIdIt];
            symmetry_linear.push_back(symmetry_TMP[symId]);
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloudHighRes, centroid);
            referencePoints_linear.push_back(centroid.head(3));
        }
        // Merge similar symmetries
        std::vector<int> symmetryMergedGlobalIds_linear;
        sym::mergeDuplicateRotSymmetries(symmetry_linear,
                                         referencePoints_linear,
                                         symmetryMergedGlobalIds_linear);
        symmetry_linear.resize(symmetryMergedGlobalIds_linear.size());
        symmetries = symmetry_linear;
        symmetryFilteredIds_TMP = symmetryMergedGlobalIds_linear;
        std::cout << "Merged symmetries: " << symmetry_linear.size() << std::endl;
        if (symmetryFilteredIds_TMP.size() > 0)
        {
            // Found symmetries
            for (size_t symId = 0; symId < symmetryFilteredIds_TMP.size(); symId++)
            {
                symmetries[symId] = symmetry_TMP[symmetryFilteredIds_TMP[symId]];
            }
            rot_symmetries.resize(symmetryFilteredIds_TMP.size());
            rot_symmetries = symmetries;
        }
    }
    template <typename PointT>
    void reflectionalDetection(sym::ReflSymDetectParams &refl_parameters)
    {
        std::vector<sym::ReflectionalSymmetry> symmetry_TMP;
        std::vector<sym::ReflectionalSymmetry> symmetries;
        std::vector<int> symmetryFilteredIds_TMP;
        sym::ReflSymDetectParams reflDetParams;
        reflDetParams = refl_parameters;
        sym::ReflectionalSymmetryDetection<PointT> rsd(reflDetParams);
        rsd.setInputCloud(cloudHighRes);
        rsd.detect();
        rsd.filter();
        rsd.getSymmetries(symmetry_TMP, symmetryFilteredIds_TMP);
        // Linearize symmetry data
        std::vector<sym::ReflectionalSymmetry> symmetry_linear;
        std::vector<Eigen::Vector3f> referencePoints_linear;

        for (size_t symIdIt = 0; symIdIt < symmetryFilteredIds_TMP.size(); symIdIt++)
        {
            int symId = symmetryFilteredIds_TMP[symIdIt];
            symmetry_linear.push_back(symmetry_TMP[symId]);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloudHighRes, centroid);
            referencePoints_linear.push_back(centroid.head(3));
        }
        // Merge similar symmetries
        std::vector<int> symmetryMergedGlobalIds_linear;
        sym::mergeDuplicateReflSymmetries(symmetry_linear,
                                          referencePoints_linear,
                                          symmetryMergedGlobalIds_linear,
                                          reflDetParams.symmetry_min_angle_diff,
                                          reflDetParams.symmetry_min_distance_diff,
                                          reflDetParams.max_reference_point_distance);
        symmetry_linear.resize(symmetryMergedGlobalIds_linear.size());
        symmetries = symmetry_linear;
        symmetryFilteredIds_TMP = symmetryMergedGlobalIds_linear;
        if (symmetryFilteredIds_TMP.size() > 0)
        {
            // Found symmetries
            for (size_t symId = 0; symId < symmetryFilteredIds_TMP.size(); symId++)
            {
                symmetries[symId] = symmetry_TMP[symmetryFilteredIds_TMP[symId]];
            }
            ref_symmetries.resize(symmetryFilteredIds_TMP.size());
            ref_symmetries = symmetries;
        }
    }
    std::vector<sym::RotationalSymmetry> getRotational(void)
    {
        return rot_symmetries;
    }
    std::vector<sym::ReflectionalSymmetry> getReflectional(void)
    {
        return ref_symmetries;
    }

};

#endif // SYMMETRY_DETECTION_HPP