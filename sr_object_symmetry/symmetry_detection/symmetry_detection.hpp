#include "filesystem/filesystem.hpp"
#include "eigen.hpp"
#include <pointcloud/pointcloud.hpp>
#include <rotational_symmetry_detection.hpp>
#include <reflectional_symmetry_detection.hpp>
#include "pointcloud/mesh_to_cloud.hpp"


class Sym
{
    public:
        Sym(string file, int sampling);
        ~Sym();
        mytype RotationalDetection();
        mytype ReflectionalDetection();

}
typedef pcl::PointXYZRGBNormal PointT;
//PointXYZRGBNormal
int ()
{
    std::string sceneDirname, sceneCloudFilename, fileName;
    if (argc > 1)
        fileName = argv[1];
    sceneDirname = "../sample_objects";
    std::cout << "Loading data..." << std::endl;
    sceneCloudFilename = utl::fullfile(sceneDirname, fileName);
    pcl::PointCloud<PointT>::Ptr sceneCloudHighRes(new pcl::PointCloud<PointT>);
    sceneCloudHighRes = convertPlyToCloud(sceneCloudFilename, 15000);
    std::vector<sym::RotationalSymmetry> symmetry_TMP;
    std::vector<int> symmetryFilteredIds_TMP;
    std::vector<int> symmetryMergedIds_TMP;
    std::vector<float> symmetryScores_TMP;
    std::vector<float> perpendicularScores_TMP;
    std::vector<float> coverageScores_TMP;
    // Rotatioanal symmetry detection parameters
    sym::RotSymDetectParams rotDetParams;
    rotDetParams.ref_max_fit_angle = pcl::deg2rad(45.0f);
    rotDetParams.min_normal_fit_angle = pcl::deg2rad(10.0f);
    rotDetParams.max_normal_fit_angle = pcl::deg2rad(60.0f);
    rotDetParams.max_symmetry_score = 0.02f;
    rotDetParams.max_perpendicular_score = 0.6f;
    rotDetParams.min_coverage_score = 0.3f;
    // Reflectional symmetry variables
    std::vector<sym::ReflectionalSymmetry> Refsymmetry_TMP;
    std::vector<int> RefsymmetryFilteredIds_TMP;
    std::vector<int> RefsymmetryMergedIds_TMP;
    std::vector<float> RefcloudInlierScores_TMP;
    std::vector<float> RefcorrespInlierScores_TMP;

    // Reflectional symmetry detection parameters
    sym::ReflSymDetectParams reflDetParams;
    reflDetParams.voxel_size = 0.0f;
    reflDetParams.num_angle_divisions = 20;
    reflDetParams.flatness_threshold = 0.005f;
    reflDetParams.refine_iterations = 20;
    reflDetParams.max_correspondence_reflected_distance = 0.01f;
    reflDetParams.min_inlier_normal_angle = pcl::deg2rad(15.0f);
    reflDetParams.max_inlier_normal_angle = pcl::deg2rad(20.0f);
    reflDetParams.min_cloud_inlier_score = 0.15f;
    reflDetParams.min_corresp_inlier_score = 0.90f;
    reflDetParams.symmetry_min_angle_diff = pcl::deg2rad(7.0);
    reflDetParams.symmetry_min_distance_diff = 0.01f;
    reflDetParams.max_reference_point_distance = 0.3f;

    // First check for rotational symmetry
    std::cout << "Looking for rotational symmetries" << std::endl;
    sym::RotationalSymmetryDetection<PointT> rsd(rotDetParams);
    rsd.setInputCloud(sceneCloudHighRes);
    rsd.detect();
    rsd.filter();
    rsd.getSymmetries(symmetry_TMP, symmetryFilteredIds_TMP, symmetryMergedIds_TMP);
    rsd.getScores(symmetryScores_TMP, perpendicularScores_TMP, coverageScores_TMP);
    std::cout << "Rotational symmetries: " << symmetryFilteredIds_TMP.size() << std::endl;
    std::cout << "Looking for reflectional symmetries" << std::endl;
    sym::ReflectionalSymmetryDetection<PointT> Refrsd(reflDetParams);
    Refrsd.setInputCloud(sceneCloudHighRes);
    Refrsd.detect();
    Refrsd.filter();
    Refrsd.getSymmetries(Refsymmetry_TMP, RefsymmetryFilteredIds_TMP, RefsymmetryMergedIds_TMP);
    Refrsd.getScores(RefcloudInlierScores_TMP, RefcorrespInlierScores_TMP);
    std::cout << "Reflectional Symmetries:" << RefsymmetryFilteredIds_TMP.size() << std::endl;
    
}
