#include "common.h"
#include <pcl/io/ascii_io.h>
#include <fstream>
#ifndef BN_POINT_CLOUD_READER_H
#define BN_POINT_CLOUD_READER_H


class BNPointCloudReader
{
public:
    BNPointCloudReader(std::string filePath,pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudToPopulate);
    void SetFileName(std::string filePath);
    void ReadPointCloud();
private:
    std::string m_pointCloudFilePath;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloudRef;
    void ReadPCDPointCloud();
    void ReadTxtPointCloud();
};

#endif