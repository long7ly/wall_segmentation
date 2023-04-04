import open3d as o3d

source = "./point_cloud_data/ply_wall/result1.pcd"
pcd = o3d.io.read_point_cloud(source)
print("pcd:", pcd)
o3d.visualization.draw_geometries([pcd])