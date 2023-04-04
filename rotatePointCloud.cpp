#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
	std::cout << "Picking event active" << std::endl;
	if (event.getPointIndex() != -1)
	{
		float x, y, z;
		event.getPoint(x, y, z);
		std::cout << x << "; " << y << "; " << z << std::endl;
	}
}

int main(int argc, char* argv[]) {

	if (argc == 1) {
		std::cout << "Need one PointCloud file! " << endl;
		return 0;
	}


	// 读取点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::string PointCloudFileName = argv[3];
	pcl::io::loadPLYFile(PointCloudFileName, *cloud);
	//pcl::io::loadPCDFile<pcl::PointXYZ>(PointCloudFileName, *cloud);
	std::cout << "Loaded " << cloud->size() << " data points from " << PointCloudFileName << std::endl;


	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
	viewer.addCoordinateSystem(0.5);
	viewer.registerPointPickingCallback(pp_callback, (void*)&viewer);
	viewer.spin();

	return 0;
}
