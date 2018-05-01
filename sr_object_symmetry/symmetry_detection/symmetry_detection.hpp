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
        std::vector<int> symmetryFilteredIds_TMP;
        Eigen::Vector3f referencePoints_linear;

        sym::RotationalSymmetryDetection<PointT> rsd(rot_parameters);
        rsd.setInputCloud(cloudHighRes);
        rsd.detect();
        rsd.filter();
        rsd.getSymmetries(symmetry_TMP, symmetryFilteredIds_TMP);
        // Merge
        if (symmetryFilteredIds_TMP.size() > 0)
        {

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloudHighRes, centroid);
            referencePoints_linear = centroid.head(3);

            // Merge similar symmetries
            std::vector<int> symmetryMergedGlobalIds_linear;
            sym::mergeDuplicateRotSymmetries(symmetry_TMP,
                                             referencePoints_linear,
                                             symmetryMergedGlobalIds_linear);
            symmetry_TMP.resize(symmetryMergedGlobalIds_linear.size());
            rot_symmetries = symmetry_TMP;
            symmetryFilteredIds_TMP = symmetryMergedGlobalIds_linear;
            if (symmetryFilteredIds_TMP.size() > 0)
            {
                rot_symmetries.resize(symmetryFilteredIds_TMP.size());
                // Found symmetries
                for (size_t symId = 0; symId < symmetryFilteredIds_TMP.size(); symId++)
                {
                    rot_symmetries[symId] = symmetry_TMP[symmetryFilteredIds_TMP[symId]];
                }
            }
        }
    }
    template <typename PointT>
    void reflectionalDetection(sym::ReflSymDetectParams &refl_parameters)
    {
        std::vector<sym::ReflectionalSymmetry> symmetry_TMP;
        std::vector<int> symmetryFilteredIds_TMP;
        Eigen::Vector3f referencePoints_linear;

        sym::ReflectionalSymmetryDetection<PointT> rsd(refl_parameters);
        rsd.setInputCloud(cloudHighRes);
        rsd.detect();
        rsd.filter();
        rsd.getSymmetries(symmetry_TMP, symmetryFilteredIds_TMP);
        // Merge
        if (symmetryFilteredIds_TMP.size() > 0)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloudHighRes, centroid);
            referencePoints_linear = centroid.head(3);

            // Merge similar symmetries
            std::vector<int> symmetryMergedGlobalIds_linear;
            sym::mergeDuplicateReflSymmetries(symmetry_TMP,
                                              referencePoints_linear,
                                              symmetryMergedGlobalIds_linear,
                                              refl_parameters.symmetry_min_angle_diff,
                                              refl_parameters.symmetry_min_distance_diff,
                                              refl_parameters.max_reference_point_distance);
            symmetry_TMP.resize(symmetryMergedGlobalIds_linear.size());
            ref_symmetries = symmetry_TMP;
            symmetryFilteredIds_TMP = symmetryMergedGlobalIds_linear;
            if (symmetryFilteredIds_TMP.size() > 0)
            {
                ref_symmetries.resize(symmetryFilteredIds_TMP.size());
                // Found symmetries
                for (size_t symId = 0; symId < symmetryFilteredIds_TMP.size(); symId++)
                {
                    ref_symmetries[symId] = symmetry_TMP[symmetryFilteredIds_TMP[symId]];
                }
            }
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