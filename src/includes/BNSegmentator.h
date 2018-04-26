#include "common.h"
#include "BNModel.h"
#include "BNLabelStore.h"
#include <pcl/search/search.h>

#include <pcl/segmentation/region_growing_rgb.h>

#ifndef BN_SEGMENTATOR_H
#define BN_SEGMENTATOR_H


class BNSegmentator
{
public:
    BNSegmentator(BNModel& inModel, BNLabelStore& inLabelStore);
    void AddLabel2ClusterMapping(uint classID,uint clusterID);
    void AnnotatePointCluster(pcl::PointXYZRGB inPoint);
    void ResegmentPointCluster(pcl::PointXYZRGB inPoint, uint mode);
    void AutoCompleteLabelling();
    void UpdateLabelledPointCloud();
private:
	void InitSegmentator();
	void SegmentPointCloud();
	void DoColorRegionGrowingSegmentation();
    void DoNormalRegionGrowingSegmentation();
	void GetClusterFromPoint();
	uint FindClusterIDFromClusters(pcl::PointIndices inCluster);
    uint FindClusterIDFromPointIndex(int inPointIndex);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloud;
    BNModel& m_model;
    std::vector <pcl::PointIndices> m_clusters;
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> m_regionGrowingSegmentatorRGB;
    pcl::RegionGrowing<pcl::PointXYZRGB,  pcl::Normal> m_regionGrowingSegmentatorN;
    pcl::MinCutSegmentation<pcl::PointXYZRGB> m_minCutSegmentator;
    std::unordered_map<uint,std::vector<uint>> m_label2ClusterMap;
    BNLabelStore& m_labelStore;
    pcl::search::KdTree<pcl::PointXYZRGB> m_searchKDTree;
    pcl::IndicesPtr m_origCloudIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> m_euclideanSegmentator;
};

#endif