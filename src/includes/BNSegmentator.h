#include "common.h"
#include "BNModel.h"
#include "BNLabelStore.h"
#include <pcl/search/search.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#ifndef BN_SEGMENTATOR_H
#define BN_SEGMENTATOR_H


class BNSegmentator
{
public:
    BNSegmentator(BNModel& inModel, BNLabelStore& inLabelStore, CONFIG& inConfig);
    void AddLabel2ClusterMapping(int classID,uint clusterID);
    void AnnotatePointCluster(pcl::PointXYZRGB inPoint);
    void ResegmentPointCluster(pcl::PointXYZRGB inPoint, uint mode);
    void AutoCompleteLabelling();
    void UpdateLabelledPointCloud();
    void WritePointCloudToFile();
    void UpdateNormalBasedSegmentatorParams(double smoothnessThreshold, double curvatureThreshold);
    void RemoveDominantPlane();
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
    pcl::SACSegmentation<pcl::PointXYZRGB> m_planeSegmentator;
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> m_regionGrowingSegmentatorRGB;
    pcl::RegionGrowing<pcl::PointXYZRGB,  pcl::Normal> m_regionGrowingSegmentatorN;
    pcl::MinCutSegmentation<pcl::PointXYZRGB> m_minCutSegmentator;
    std::unordered_map<int,std::vector<uint>> m_label2ClusterMap;
    BNLabelStore& m_labelStore;
    pcl::search::KdTree<pcl::PointXYZRGB> m_searchKDTree;
    pcl::IndicesPtr m_origCloudIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> m_euclideanSegmentator;
    CONFIG& m_prgConfig;
};

#endif