#include "common.h"
#include "BNModel.h"

#include <pcl/segmentation/region_growing_rgb.h>

#ifndef BN_SEGMENTATOR_H
#define BN_SEGMENTATOR_H


class BNSegmentator
{
public:
    BNSegmentator(BNModel& inModel);
    
private:
	void InitSegmentator();
	void SegmentPointCloud();
	void DoRegionGrowingSegmentation();
	void GetClusterFromPoint();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloud;
    BNModel& m_model;
    std::vector <pcl::PointIndices> m_clusters;
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> m_regionGrowingSegmentator;
};

#endif