#ifndef SYMMETRY_DETECTION_HPP
#define SYMMETRY_DETECTION_HPP

#include "eigen.hpp"
#include <pointcloud/pointcloud.hpp>
#include <rotational_symmetry_detection.hpp>
#include <reflectional_symmetry_detection.hpp>
#include "pointcloud/mesh_to_cloud.hpp"

typedef pcl::PointXYZRGBNormal PointT;
pcl::PointCloud<PointT>::Ptr cloudHighRes(new pcl::PointCloud<PointT>);

void loadFile(std::string filename, int sampling)
{
    cloudHighRes = convertPlyToCloud(filename, sampling);
}
template <typename PointT>
bool RotationalDetection(sym::RotSymDetectParams &rot_parameters, std::vector<sym::RotationalSymmetry> &symmetries)
{
    std::vector<sym::RotationalSymmetry> symmetry_TMP;
    std::vector<int> symmetryFilteredIds_TMP;
    sym::RotSymDetectParams rotDetParams;
    rotDetParams = rot_parameters;
    sym::RotationalSymmetryDetection<PointT> rsd(rotDetParams);
    rsd.setInputCloud(cloudHighRes);
    rsd.detect();
    rsd.filter();
    rsd.getSymmetries(symmetry_TMP, symmetryFilteredIds_TMP);
    if (symmetryFilteredIds_TMP.size() > 0)
    {
        // Found symmetries
        symmetries.resize(symmetryFilteredIds_TMP.size());
        for (size_t symId = 0; symId < symmetryFilteredIds_TMP.size(); symId++)
        {
            symmetries[symId] = symmetry_TMP[symmetryFilteredIds_TMP[symId]];
        }
        return true;
    }
    else
        return false;
}
template <typename PointT>
bool ReflectionalDetection(sym::ReflSymDetectParams &refl_parameters, std::vector<sym::ReflectionalSymmetry> &symmetries)
{
    std::vector<sym::ReflectionalSymmetry> symmetry_TMP;
    std::vector<int> symmetryFilteredIds_TMP;
    sym::ReflSymDetectParams reflDetParams;
    reflDetParams = refl_parameters;
    sym::ReflectionalSymmetryDetection<PointT> rsd(reflDetParams);
    rsd.setInputCloud(cloudHighRes);
    rsd.detect();
    rsd.filter();
    rsd.getSymmetries(symmetry_TMP, symmetryFilteredIds_TMP);
    if (symmetryFilteredIds_TMP.size() > 0)
    {
        // Found symmetries
        symmetries.resize(symmetryFilteredIds_TMP.size());
        for (size_t symId = 0; symId < symmetryFilteredIds_TMP.size(); symId++)
        {
            symmetries[symId] = symmetry_TMP[symmetryFilteredIds_TMP[symId]];
        }
        return true;
    }
    else
        return false;
}
#endif // SYMMETRY_DETECTION_HPP