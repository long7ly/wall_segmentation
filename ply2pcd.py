import open3d as o3d
import os
import re

def get_point_cloud_files(folder_path):
    source_cloud_files = []
    target_cloud_files = []

    for file_name in os.listdir(folder_path):
        source_cloud_files.append(folder_path + "/" + file_name)
        file_name = re.sub("ply","pcd",file_name)
        target_cloud_files.append(folder_path + "/" + file_name)    
    print("source_cloud_files:", source_cloud_files)
    print("target_cloud_files:", target_cloud_files)
    return source_cloud_files,target_cloud_files

path = "./point_cloud_data/ply_wall"
source, target = get_point_cloud_files(path)

for i in range(0, len(source)):
    pcd = o3d.io.read_point_cloud(source[i])
    o3d.io.write_point_cloud(target[i], pcd)