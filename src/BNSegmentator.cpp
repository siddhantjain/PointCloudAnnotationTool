#include "includes/BNSegmentator.h"

BNSegmentator::BNSegmentator(BNModel& inModel):
m_model(inModel)
{
    cout << "Constructing Segmentator" << endl;
    InitSegmentator();
}

void BNSegmentator::InitSegmentator()
{
    //siddhant: Check if we really need this indices business? 
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;

    pass.setInputCloud (m_model.GetRawPointCloud());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 10.0);
    pass.filter (*indices);

    m_regionGrowingSegmentator.setInputCloud (m_model.GetRawPointCloud());
    m_regionGrowingSegmentator.setIndices (indices);
    m_regionGrowingSegmentator.setSearchMethod (tree);
    m_regionGrowingSegmentator.setDistanceThreshold (10);
    m_regionGrowingSegmentator.setPointColorThreshold (6);
    m_regionGrowingSegmentator.setRegionColorThreshold (5);
    m_regionGrowingSegmentator.setMinClusterSize (600); 

    SegmentPointCloud();
    cout << "Segmentation Initialisation done" << endl;
}

void BNSegmentator::SegmentPointCloud()
{
    DoRegionGrowingSegmentation();
}
void BNSegmentator::DoRegionGrowingSegmentation()
{
    cout << "doing region growing segmentation" << endl;
    m_regionGrowingSegmentator.extract(m_clusters);
    m_model.SetSegmentedPointCloud(m_regionGrowingSegmentator.getColoredCloud());
}