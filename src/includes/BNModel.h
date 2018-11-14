#include "common.h"
#include "BNState.h"
#include "BNLabelStore.h"

#ifndef BN_MODEL_H
#define BN_MODEL_H


class BNModel
{
public:
    BNModel(BNState& inState, BNLabelStore& inStore,CONFIG& inConfig);
    void InitModel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inPointCloud);
    void SetPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inPointCloud);
    void SetSegmentedPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inSPointCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetRawPointCloud ();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetLabelledPointCloud ();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetSegmentedPointCloud ();
    std::string GetState();
    void SetState(std::string inState);
    int GetAnnotationClass();
    void SetAnnotationClass(int inClass);
    BNLabelStore& GetLabelStore();
    void WriteLabelledPointCloud(bool writeAllPoints=false);
    void GetNewLabels();
    std::vector<bool> isHumanAnnotated;
    void CallPythonFineTune(bool updateModel=false);
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_labelledPointCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_segmentedPointCloud;
    void InitLabels();
    BNState& m_state;
    CONFIG& m_config;
    //siddhant: This is a temp. The purpose is to serve as the number of class the user is annotating it
    //need to spend some time thinking about the best way to implement this
    int m_annotationClassNum;
    BNLabelStore& m_labelStore;

};

#endif