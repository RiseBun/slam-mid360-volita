//
// Created by xiang on 2022/3/15.
//

#ifndef SLAM_IN_AUTO_DRIVING_LIDAR_UTILS_H
#define SLAM_IN_AUTO_DRIVING_LIDAR_UTILS_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include "cloudMap.hpp"
#include "tools/point_types.h"

namespace zjloc
{
    FullCloudPtr Convert3DtoCloud(const std::vector<point3D> &input)
    {
        FullCloudPtr cloud(new FullPointCloudType);
        for (auto pt : input)
        {
            FullPointType p;
            p.x = pt.raw_point[0];
            p.y = pt.raw_point[1];
            p.z = pt.raw_point[2];
            p.intensity = pt.intensity;
            cloud->points.template emplace_back(p);
        }
        cloud->width = input.size();
        return cloud;
    }
    /**
     * 其他类型点云转到PointType点云
     * 用的最多的是全量点云转到XYZI点云
     * @tparam PointT
     * @param input
     * @return
     */
    template <typename PointT = FullPointType>
    CloudPtr ConvertToCloud(typename pcl::PointCloud<PointT>::Ptr input)
    {
        CloudPtr cloud(new PointCloudType);
        for (auto &pt : input->points)
        {
            PointType p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = pt.intensity;
            cloud->points.template emplace_back(p);
        }
        cloud->width = input->width;
        return cloud;
    }

    /// 对点云进行voxel filter,指定分辨率
    /// 使用坐标中心化避免 int32 索引溢出
    inline CloudPtr VoxelCloud(CloudPtr cloud, float voxel_size = 0.1)
    {
        if (!cloud || cloud->empty())
            return cloud;

        // 计算点云质心，用于中心化
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);

        // 将点云平移到原点附近（避免 VoxelGrid int32 溢出）
        CloudPtr shifted(new PointCloudType);
        shifted->reserve(cloud->size());
        for (const auto &pt : cloud->points)
        {
            PointType p;
            p.x = pt.x - centroid[0];
            p.y = pt.y - centroid[1];
            p.z = pt.z - centroid[2];
            p.intensity = pt.intensity;
            shifted->points.push_back(p);
        }

        // 执行体素滤波
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel.setInputCloud(shifted);

        CloudPtr filtered(new PointCloudType);
        voxel.filter(*filtered);

        // 将结果平移回原坐标系
        CloudPtr output(new PointCloudType);
        output->reserve(filtered->size());
        for (const auto &pt : filtered->points)
        {
            PointType p;
            p.x = pt.x + centroid[0];
            p.y = pt.y + centroid[1];
            p.z = pt.z + centroid[2];
            p.intensity = pt.intensity;
            output->points.push_back(p);
        }
        output->width = output->size();
        output->height = 1;

        return output;
    }

} // namespace zjloc

#endif // SLAM_IN_AUTO_DRIVING_LIDAR_UTILS_H
