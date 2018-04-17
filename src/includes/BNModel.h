#include "common.h"
#include "BNState.h"

#ifndef BN_MODEL_H
#define BN_MODEL_H


class BNModel
{
public:
    BNModel(BNState& inState);
    void InitModel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inPointCloud);
    void SetPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inPointCloud);
    void SetSegmentedPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inSPointCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetRawPointCloud ();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetLabelledPointCloud ();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetSegmentedPointCloud ();
    std::string GetState();
    void SetState(std::string inState);
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_labelledPointCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_segmentedPointCloud;
    void InitLabels();
    BNState& m_state;
};

#endif