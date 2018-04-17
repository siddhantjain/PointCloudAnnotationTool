#include "includes/BNModel.h"

BNModel::BNModel(BNState& inState):
m_state(inState)
{
    std::cout << "Model Created" << std::endl;
}
void BNModel::InitModel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inPointCloud)
{
    m_pointCloud = inPointCloud;
    m_labelledPointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB> );
    m_segmentedPointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::copyPointCloud(*m_pointCloud,*m_labelledPointCloud);
    InitLabels();
}
void BNModel::InitLabels()
{
    //change the colors of each point indice to the same color
    //siddhant: There should be faster/more efficient way of doing this
    for(int i=0;i<m_labelledPointCloud->points.size();i++)
    {
        m_labelledPointCloud->points[i].r = 255;
        m_labelledPointCloud->points[i].g = 255;
        m_labelledPointCloud->points[i].b = 255;
    }
}
void BNModel::SetPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inPointCloud)
{
    m_pointCloud = inPointCloud;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr BNModel::GetRawPointCloud ()
{
    //siddhant: add a null check here
    return m_pointCloud;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr BNModel::GetLabelledPointCloud ()
{
    //siddhant: add a null check here
    return m_labelledPointCloud;
} 
void BNModel::SetSegmentedPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inSPointCloud)
{
    pcl::copyPointCloud(*inSPointCloud,*m_segmentedPointCloud);
    cout << "Number of points in Segemented Point Cloud: " << m_segmentedPointCloud->points.size() << endl;
    cout << "Number of points in input Point Cloud: " << inSPointCloud->points.size() << endl;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr BNModel::GetSegmentedPointCloud()
{
    return m_segmentedPointCloud;
}
std::string BNModel::GetState()
{

    return m_state.GetState();
}
void BNModel::SetState(std::string inState)
{

    m_state.SetState(inState);
}