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
// 用于区域生长
#include <pcl/segmentation/region_growing.h>
// 用于搜索
#include <pcl/search/search.h>
// 用于滤波
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

int main() {
    // 读取点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string PointCloudFileName = "0_1.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ>(PointCloudFileName, *cloud);
    std::cout << "Loaded " << cloud->size() << " data points from " << PointCloudFileName << std::endl;
    
    /*
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "raw_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "raw_cloud");
    viewer.spin();
    */

    // 创建滤波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(20);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    std::cout << "has " << cloud_filtered->size() << " points after Statistical Filter" << std::endl;

    pcl::PassThrough<pcl::PointXYZ> XYZpass;
    XYZpass.setInputCloud(cloud_filtered);
    XYZpass.setFilterFieldName("z");
    XYZpass.setFilterLimits(-4, 4);
    XYZpass.filter(*cloud_filtered);
    std::cout << "has " << cloud_filtered->size() << " points after XYZ Filter" << std::endl;
    /*
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud_filtered");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_filtered");
    viewer.spin();
    */

    // 移除已显示点云
    //viewer.removeAllPointClouds();

    // 下采样
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud_filtered);
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

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*downsampled_cloud, *normals, *cloud_normals);

    pcl::PassThrough<pcl::PointNormal> pass;
    pass.setInputCloud(cloud_normals);
    pass.setFilterFieldName("normal_y");
    pass.setFilterLimits(-0.8, 0.8);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_filtered(new pcl::PointCloud<pcl::PointNormal>);
    pass.filter(*cloud_normals_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Normfiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_Normfiltered(new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud(*cloud_normals_filtered, *cloud_Normfiltered);
    pcl::copyPointCloud(*cloud_normals_filtered, *normals_Normfiltered);
    std::cout << "have " << cloud_Normfiltered->size() << " points after Normal Filter" << std::endl;

    /*
    pcl::visualization::PCLVisualizer TempViewer("temp");
    TempViewer.setBackgroundColor(0, 0, 0);
    TempViewer.addPointCloud<pcl::PointXYZ>(cloud_Normfiltered, "temp");
    TempViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "temp");
    TempViewer.spin();
    */

    // 设置区域生长算法参数
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(1000);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud_Normfiltered);
    //reg.setIndices (indices);
    reg.setInputNormals(normals_Normfiltered);
    reg.setSmoothnessThreshold(10.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(0.09);

    // 区域分割
    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "Found " << clusters.size() << " clusters" << std::endl;

    // 可视化
    //原点云可视
    pcl::visualization::PCLVisualizer Rawviewer("Raw Cloud");
    Rawviewer.setBackgroundColor(0, 0, 0);
    Rawviewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    Rawviewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
    //分割点云可视化
    pcl::visualization::PCLVisualizer viewer("Segment Cloud");
    viewer.setBackgroundColor(0, 0, 0);
    //viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
    for (size_t i = 0; i < clusters.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_Normfiltered, clusters[i], *cluster);

        std::stringstream ss;
        ss << "cluster_" << i;
        std::cout << ss.str() << " has " << cluster->size() << " points" << std::endl;
        

        viewer.addPointCloud(cluster, ss.str());//
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, ss.str());
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)i / clusters.size(), 0.0, 1.0 - (double)i / clusters.size(), ss.str());
    }
    viewer.addCoordinateSystem(0.5);
    std::cout << "display " << clusters.size() << " clusters" << std::endl;
    viewer.spin();
    
    /*
    // 使用Ransac方法检测每个区域中的平面
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    for (int i = 0; i < clusters.size(); i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setInputCloud(cloud);
        extract.setIndices(boost::make_shared<pcl::PointIndices>(clusters[i]));
        extract.setNegative(false);
        extract.filter(*cluster_cloud);
        seg.setInputCloud(cluster_cloud);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.segment(*inliers, *coefficients);
        std::cout << "Plane " << i << " coefficients: " << coefficients->values[0] << " "
            << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
    }
    */

    return 0;
}