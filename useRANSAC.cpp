#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> setSegmentationParameter(pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg)
{
    // 设置 RANSAC 参数
    float distance_threshold = 0.016;
    int ransac_n = 3;
    int num_iterations = 1000;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(num_iterations);
    seg.setDistanceThreshold(distance_threshold);

    return seg;
}

int main(int argc, char** argv)
{
    // 读取点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("1.pcd", *cloud);
    std::cout << "Loaded " << cloud->size() << " data points from test_pcd.pcd" << std::endl;

    // 下采样
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.001f, 0.001f, 0.001f);
    vg.filter(*downsampled_cloud);
    std::cout << "Downsampled cloud contains " << downsampled_cloud->size() << " data points" << std::endl;
    
    // 计算法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(downsampled_cloud);
    ne.setInputCloud(downsampled_cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(30);
    ne.compute(*normals);
    std::cout << "Computed " << normals->size() << " normals" << std::endl;

    // 平面分割参数
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg = setSegmentationParameter(seg);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // 存储所有检测到的墙面
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> all_walls;
    int wall_num = 0;

    // 循环迭代，提取所有墙面
    while (true)
    {
        // 平面分割
        seg.setInputCloud(downsampled_cloud);
        seg.setInputNormals(normals);
        seg.segment(*inliers, *coefficients);
        std::cout << "Found " << inliers->indices.size() << " inliers" << std::endl;

        // 检查是否检测到平面
        if (inliers->indices.size() < 5000) {
            break;
        }

        // 储存平面点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

        // 提取平面模型内的点云数据
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(downsampled_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);
        // 将墙面储存
        //std::cout << "this plane has " << cloud_plane->size() << " points" << std::endl;
        all_walls.push_back(cloud_plane);
        std::cout << "Downsampled cloud contains " << downsampled_cloud->size() << " data points" << std::endl;
        std::cout << "already detect " << all_walls.size() << " walls" << std::endl;


        // 移除平面对应的点云
        extract.setNegative(true);
        extract.filter(*downsampled_cloud);
        // 移除对应点云对应的法向量
        pcl::ExtractIndices<pcl::Normal> extract_normals;  // 使用ExtractIndices类
        extract_normals.setInputCloud(normals);
        extract_normals.setIndices(inliers);
        extract_normals.setNegative(true);
        extract_normals.filter(*normals);  // 将平面以及对应的法向量从点云中滤除
        

        /*// 获取平面参数
        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];
        std::cout << "a:" << a << "b:" << b << "c:" << c << std::endl;
        */
        //break;
    }

    std::vector<Eigen::Vector3i> colors;

    // 可视化
    pcl::visualization::PCLVisualizer viewer("Plane Extraction");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
    
    for (size_t i = 0; i < all_walls.size(); ++i) {
        std::stringstream ss;
        ss << "plane_" << i;
        std::cout << ss.str() << "has" << all_walls[i]->size() << "points" << std::endl;
        viewer.addPointCloud(all_walls[i], ss.str());//
        //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());
        //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, ss.str());
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)i / all_walls.size(), 0.0, 1.0 - (double)i / all_walls.size(), ss.str());
    }
    viewer.addCoordinateSystem(1.0);
    std::cout << "display " << all_walls.size() << " walls" << std::endl;
    viewer.spin();

    //viewer.addPlane(*coefficients, "plane");
    //viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "plane");
    //viewer.spin();

    return 0;
}