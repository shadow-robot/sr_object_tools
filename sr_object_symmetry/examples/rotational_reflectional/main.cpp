#include "filesystem/filesystem.hpp"
#include "eigen.hpp"
#include <pointcloud/pointcloud.hpp>
#include <rotational_symmetry_detection.hpp>
#include <reflectional_symmetry_detection.hpp>
#include "vis.hpp"
#include "pointcloud/mesh_to_cloud.hpp"

typedef pcl::PointXYZRGBNormal PointT;
//PointXYZRGBNormal
int main(int argc, char **argv)
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
    
    std::cout << "Controls:" << std::endl;
    std::cout << "Numpad 1: Show Point Cloud" << std::endl;
    std::cout << "Numpad 3: Show Rotational Symmetries" << std::endl;
    std::cout << "Numpad 7: Show Reflectional Symmetries" << std::endl;
    std::cout << "Left/Right arrows: Show next symmetry in list" << std::endl;
    
    
    // Visualize
    VisState visState;
    pcl::visualization::PCLVisualizer visualizer;
    visualizer.setCameraPosition(0.0, 0.0, -1.0, // camera position
                                 0.0, 0.0, 1.0,  // viewpoint
                                 0.0, -1.0, 0.0, // normal
                                 0.0);           // viewport
    visualizer.setBackgroundColor(utl::bgColor.r, utl::bgColor.g, utl::bgColor.b);
    visualizer.registerKeyboardCallback(keyboard_callback, (void *)(&visState));
    visState.updateDisplay_ = true;

    while (!visualizer.wasStopped())
    {
        // Update display if needed
        if (visState.updateDisplay_)
        {
            // First remove everything
            visualizer.removeAllPointClouds();
            visualizer.removeAllShapes();
            visualizer.removeAllCoordinateSystems();
            visState.updateDisplay_ = false;
            if (visState.cloudDisplay_ == VisState::CLOUD)
            {
                pcl::PointCloud<PointT>::Ptr cloudDisplay(new pcl::PointCloud<PointT>);

                std::string text;
                if (visState.cloudDisplay_ == VisState::CLOUD)
                {
                    cloudDisplay = sceneCloudHighRes;
                    text = "Original cloud";
                }
                utl::showPointCloudColor<PointT>(visualizer, cloudDisplay, "cloud", visState.pointSize_);
                if (visState.showNormals_)
                    utl::showNormalCloud<PointT>(visualizer, cloudDisplay, 10, 0.02, "normals", visState.pointSize_, utl::green);

                visualizer.addText(text, 0, 150, 24, 1.0, 1.0, 1.0);
            }
            else if (visState.cloudDisplay_ == VisState::ROTATIONAL_SYMMETRIES)
            {
                std::vector<sym::RotationalSymmetry> symmetryDisplay;
                std::vector<int> symmetryDisplayIds;
                std::string text;
                symmetryDisplay = symmetry_TMP;
                text = "Rotational symmetries";
                for (size_t symId = 0; symId < symmetryFilteredIds_TMP.size(); symId++)
                {
                    symmetryDisplayIds.push_back(symmetryFilteredIds_TMP[symId]);
                }
                visState.segIterator_ = utl::clampValueCircular<int>(visState.segIterator_, 0, symmetryDisplayIds.size() - 1);
                int symId = symmetryDisplayIds[visState.segIterator_];
                utl::showPointCloudColor<PointT>(visualizer, sceneCloudHighRes, "cloud", visState.pointSize_);
                // Show symmetry
                if (visState.showSymmetry_)
                    sym::showRotationalSymmetry(visualizer, symmetryDisplay[symId], "symmetry", 5, 5.0);
                visualizer.addText(text, 0, 150, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Symmetry " + std::to_string(visState.segIterator_ + 1) + " / " + std::to_string(symmetryDisplayIds.size()), 15, 125, 24, 1.0, 1.0, 1.0);
            }
            else if (visState.cloudDisplay_ == VisState::REFLECTIONAL_SYMMETRIES)
            {
                std::vector<sym::ReflectionalSymmetry> symmetryDisplay;
                std::vector<int> symmetryDisplayIds;
                std::string text;

                if (visState.cloudDisplay_ == VisState::REFLECTIONAL_SYMMETRIES)
                {
                    symmetryDisplay = Refsymmetry_TMP;
                    for (size_t symId = 0; symId < RefsymmetryFilteredIds_TMP.size(); symId++)
                        symmetryDisplayIds.push_back(RefsymmetryFilteredIds_TMP[symId]);
                    text = "Reflectional symmetry ";
                }

                visState.segIterator_ = utl::clampValueCircular<int>(visState.segIterator_, 0, symmetryDisplayIds.size() - 1);
                int symId = symmetryDisplayIds[visState.segIterator_];
                text = "cloudInlierScores Score: " + std::to_string(RefcloudInlierScores_TMP[symId]) + " correspInlierScores Score: " + std::to_string(RefcorrespInlierScores_TMP[symId]);
                utl::showPointCloudColor<PointT>(visualizer, sceneCloudHighRes, "cloud", visState.pointSize_);

                // Show symmetry
                if (visState.showSymmetry_)
                    sym::showReflectionalSymmetry(visualizer, symmetryDisplay[symId], "symmetry", 1);

                visualizer.addText(text, 0, 150, 24, 1.0, 1.0, 1.0);
                visualizer.addText("Symmetry " + std::to_string(visState.segIterator_ + 1) + " / " + std::to_string(symmetryDisplayIds.size()), 15, 125, 24, 1.0, 1.0, 1.0);
            }
        }
        // Spin once
        visualizer.spinOnce();
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
}
