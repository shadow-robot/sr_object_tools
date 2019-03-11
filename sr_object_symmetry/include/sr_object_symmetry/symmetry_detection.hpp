/**
* Copyright 2019 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SR_OBJECT_SYMMETRY_SYMMETRY_DETECTION_H
#define SR_OBJECT_SYMMETRY_SYMMETRY_DETECTION_H

#include <string>
#include <vector>

#include "utilities/eigen.hpp"
#include <utilities/pointcloud/pointcloud.hpp>
#include <sr_object_symmetry/rotational_symmetry_detection.hpp>
#include <sr_object_symmetry/reflectional_symmetry_detection.hpp>
#include "utilities/pointcloud/mesh_to_cloud.hpp"

class SymmetryDetection
{
private:
  std::vector<sym::RotationalSymmetry> rot_symmetries;
  std::vector<sym::ReflectionalSymmetry> ref_symmetries;
  typedef pcl::PointXYZRGBNormal PointT;
  pcl::PointCloud<PointT>::Ptr cloudHighRes{ new pcl::PointCloud<PointT> };
  std::vector<float> rot_symmetry_scores;
  std::vector<float> rot_perpendicular_scores;
  std::vector<float> rot_coverage_scores;
  std::vector<float> ref_cloud_inlier_scores;
  std::vector<float> ref_corresp_inlier_scores;

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
  void rotationalDetection(sym::RotSymDetectParams& rot_parameters)
  {
    std::vector<sym::RotationalSymmetry> symmetry_TMP;
    std::vector<sym::RotationalSymmetry> symmetry_filtered;

    std::vector<int> symmetryFilteredIds_TMP;
    Eigen::Vector3f referencePoints_linear;
    std::vector<float> rot_symmetry_scores_filt;
    sym::RotationalSymmetryDetection<PointT> rsd(rot_parameters);
    rsd.setInputCloud(cloudHighRes);
    rsd.detect();
    rsd.filter();
    rsd.getSymmetries(symmetry_TMP, symmetryFilteredIds_TMP);
    rsd.getScores(rot_symmetry_scores, rot_perpendicular_scores, rot_coverage_scores);
    // Merge
    if (symmetryFilteredIds_TMP.size() > 0)
    {
      symmetry_filtered.resize(symmetryFilteredIds_TMP.size());
      rot_symmetry_scores_filt.resize(symmetryFilteredIds_TMP.size());
      for (size_t symId = 0; symId < symmetryFilteredIds_TMP.size(); symId++)
      {
        symmetry_filtered[symId] = symmetry_TMP[symmetryFilteredIds_TMP[symId]];
        rot_symmetry_scores_filt[symId] = rot_symmetry_scores[symmetryFilteredIds_TMP[symId]];
      }
      // Merge
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloudHighRes, centroid);
      referencePoints_linear = centroid.head(3);

      // Merge similar symmetries
      std::vector<int> symmetryMergedGlobalIds_linear;
      sym::mergeDuplicateRotSymmetries(symmetry_filtered, referencePoints_linear, symmetryMergedGlobalIds_linear);
      rot_symmetries.resize(symmetryMergedGlobalIds_linear.size());
      rot_symmetry_scores.resize(symmetryMergedGlobalIds_linear.size());
      for (size_t symId = 0; symId < symmetryMergedGlobalIds_linear.size(); symId++)
      {
        rot_symmetries[symId] = symmetry_filtered[symmetryMergedGlobalIds_linear[symId]];
        rot_symmetry_scores[symId] = rot_symmetry_scores_filt[symmetryMergedGlobalIds_linear[symId]];
      }
    }
  }
  template <typename PointT>
  void reflectionalDetection(sym::ReflSymDetectParams& refl_parameters)
  {
    std::vector<sym::ReflectionalSymmetry> symmetry_TMP;
    std::vector<sym::ReflectionalSymmetry> symmetry_filtered;

    std::vector<int> symmetryFilteredIds_TMP;
    Eigen::Vector3f referencePoints_linear;
    std::vector<float> ref_cloud_inlier_scores_filt;
    std::vector<float> ref_corresp_inlier_scores_filt;
    sym::ReflectionalSymmetryDetection<PointT> rsd(refl_parameters);
    rsd.setInputCloud(cloudHighRes);
    rsd.detect();
    rsd.filter();
    rsd.getSymmetries(symmetry_TMP, symmetryFilteredIds_TMP);
    rsd.getScores(ref_cloud_inlier_scores, ref_corresp_inlier_scores);
    // filter symmetries
    if (symmetryFilteredIds_TMP.size() > 0)
    {
      symmetry_filtered.resize(symmetryFilteredIds_TMP.size());
      ref_cloud_inlier_scores_filt.resize(symmetryFilteredIds_TMP.size());
      ref_corresp_inlier_scores_filt.resize(symmetryFilteredIds_TMP.size());
      for (size_t symId = 0; symId < symmetryFilteredIds_TMP.size(); symId++)
      {
        symmetry_filtered[symId] = symmetry_TMP[symmetryFilteredIds_TMP[symId]];
        ref_cloud_inlier_scores_filt[symId] = ref_cloud_inlier_scores[symmetryFilteredIds_TMP[symId]];
        ref_corresp_inlier_scores_filt[symId] = ref_corresp_inlier_scores[symmetryFilteredIds_TMP[symId]];
      }
      // Merge
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloudHighRes, centroid);
      referencePoints_linear = centroid.head(3);

      // Merge similar symmetries
      std::vector<int> symmetryMergedGlobalIds_linear;
      sym::mergeDuplicateReflSymmetries(symmetry_filtered, referencePoints_linear, symmetryMergedGlobalIds_linear,
                                        refl_parameters.symmetry_min_angle_diff,
                                        refl_parameters.symmetry_min_distance_diff,
                                        refl_parameters.max_reference_point_distance);
      ref_symmetries.resize(symmetryMergedGlobalIds_linear.size());
      ref_cloud_inlier_scores.resize(symmetryMergedGlobalIds_linear.size());
      ref_corresp_inlier_scores.resize(symmetryMergedGlobalIds_linear.size());
      for (size_t symId = 0; symId < symmetryMergedGlobalIds_linear.size(); symId++)
      {
        ref_symmetries[symId] = symmetry_filtered[symmetryMergedGlobalIds_linear[symId]];
        ref_cloud_inlier_scores[symId] = ref_cloud_inlier_scores_filt[symmetryMergedGlobalIds_linear[symId]];
        ref_corresp_inlier_scores[symId] = ref_corresp_inlier_scores_filt[symmetryMergedGlobalIds_linear[symId]];
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
  std::vector<float> getRotationalScores(void)
  {
    return rot_symmetry_scores;
  }
  std::vector<float> getReflectionalScores(void)
  {
    return ref_cloud_inlier_scores;
  }
  void removeCurrRotational(int symDel)
  {
    std::vector<sym::RotationalSymmetry> rot_symmetries_tmp;
    std::vector<float> rot_symmetry_scores_tmp;
    rot_symmetry_scores_tmp.resize(0);
    rot_symmetries_tmp.resize(0);
    for (size_t symId = 0; symId < rot_symmetries.size(); symId++)
      {
        if (symId != symDel)
        {
          rot_symmetries_tmp.push_back(rot_symmetries[symId]);
          rot_symmetry_scores_tmp.push_back(rot_symmetry_scores[symId]);
        }
      }
      rot_symmetries = rot_symmetries_tmp;
      rot_symmetry_scores = rot_symmetry_scores_tmp;
  }
  void removeCurrReflectional(int symDel)
  {
    std::vector<sym::ReflectionalSymmetry> ref_symmetries_tmp;
    std::vector<float> ref_cloud_inlier_scores_tmp;
    std::vector<float> ref_corresp_inlier_scores_tmp;
    ref_symmetries_tmp.resize(0);
    ref_cloud_inlier_scores_tmp.resize(0);
    ref_corresp_inlier_scores_tmp.resize(0);
    for (size_t symId = 0; symId < ref_symmetries.size(); symId++)
      {
        if (symId != symDel)
        {
          ref_symmetries_tmp.push_back(ref_symmetries[symId]);
          ref_cloud_inlier_scores_tmp.push_back(ref_cloud_inlier_scores[symId]);
          ref_corresp_inlier_scores_tmp.push_back(ref_corresp_inlier_scores[symId]);
        }
      }
      ref_symmetries = ref_symmetries_tmp;
      ref_cloud_inlier_scores = ref_cloud_inlier_scores_tmp;
      ref_corresp_inlier_scores = ref_corresp_inlier_scores_tmp;
  }
};

#endif  // SR_OBJECT_SYMMETRY_SYMMETRY_DETECTION_H
