#include "includes/BNSegmentator.h"

BNSegmentator::BNSegmentator(BNModel& inModel, BNLabelStore& inStore):
m_model(inModel),
m_labelStore(inStore)
{
    cout << "Constructing Segmentator" << endl;
    InitSegmentator();
    //initialise cluster map based on classes stored in the model
}

void BNSegmentator::InitSegmentator()
{
    //siddhant: Check if we really need this indices business? 
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;

    //siddhant: we have two trees in the segmentator right now
    // one of them is mostly redundant. Figure out which one and resolve this stupidity
    m_searchKDTree.setInputCloud(m_model.GetRawPointCloud());

    pass.setInputCloud (m_model.GetRawPointCloud());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 10.0);
    pass.filter (*indices);

    m_regionGrowingSegmentator.setInputCloud (m_model.GetRawPointCloud());
    m_regionGrowingSegmentator.setIndices (indices);
    m_regionGrowingSegmentator.setSearchMethod (tree);
    m_regionGrowingSegmentator.setDistanceThreshold (10);
    m_regionGrowingSegmentator.setPointColorThreshold (6);
    m_regionGrowingSegmentator.setRegionColorThreshold (2);
    m_regionGrowingSegmentator.setMinClusterSize (600); 

    SegmentPointCloud();

    std::vector<BNLabel> pcLabels = m_labelStore.GetLabels();
    for(int i=0;i<pcLabels.size();i++)
    {
        std::vector<uint> linkedClusters;
        m_label2ClusterMap[pcLabels[i].m_labelID] = linkedClusters;
    }
    cout << "Segmentation Initialisation done" << endl;
}

void BNSegmentator::SegmentPointCloud()
{
    DoRegionGrowingSegmentation();
}

void BNSegmentator::DoRegionGrowingSegmentation()
{
    cout << "Doing region growing segmentation" << endl;
    m_regionGrowingSegmentator.extract(m_clusters);
    m_model.SetSegmentedPointCloud(m_regionGrowingSegmentator.getColoredCloud());
}


void HandleExistingClusterReference(std::unordered_map<uint,std::vector<uint>>& clusterMap, uint clusterID)
{
    //siddhant: Currently all I am doing to handle an existing cluster reference is to remove that reference
    //What can be done is to resegment an existing reference by a min-cut segmentation on just point cloud where the point 
    // for which a new segment is select is marked as foreground? 
    //or we can do region growing, setting the max and min number of clusters to be two?

    std::unordered_map<uint,std::vector<uint>>::iterator startIt = clusterMap.begin();
    std::unordered_map<uint,std::vector<uint>>::iterator endIt = clusterMap.end();

    while(startIt!=endIt)
    {
        std::vector<uint>& clusterIDs = startIt->second;
        std::vector<uint>::iterator foundValIt;
        foundValIt = std::find(clusterIDs.begin(), clusterIDs.end(),clusterID); 
        if (foundValIt != clusterIDs.end())
        {
            clusterIDs.erase(foundValIt);
            return;
        }

        startIt++;
    }
       
}
void BNSegmentator::AddLabel2ClusterMapping(uint labelID,uint clusterID)
{
    if(clusterID >= m_clusters.size())
    {
        cout << "invalid cluster number. No annotation done" << endl;
        return;
    }

    //if clusterID already given some other cluster value, we should remove the old mapping
    //need to think of efficient ways of doing this, as it would not scale

    HandleExistingClusterReference(m_label2ClusterMap,clusterID);


    m_label2ClusterMap[labelID].push_back(clusterID);
}
uint BNSegmentator::FindClusterIDFromClusters(pcl::PointIndices inCluster)
{
    std::vector<pcl::PointIndices>::iterator startIt = m_clusters.begin();
    std::vector <pcl::PointIndices>::iterator endIt = m_clusters.end();

    uint clusterID=0;
    while(startIt!=endIt)
    {
        if(startIt->indices == inCluster.indices)
            return clusterID;
        clusterID++;
        startIt++;
    }

    return clusterID;

}
void BNSegmentator::AnnotatePointCluster(pcl::PointXYZRGB inPoint)
{
    pcl::PointIndices cluster;
    std::vector<int> indices1 (1);
    std::vector<float> distances (1);
        

    m_searchKDTree.nearestKSearch (inPoint, 1, indices1, distances);
    m_regionGrowingSegmentator.getSegmentFromPoint(indices1[0],cluster);

    //Siddhant: I am not sure if PCL has a more efficient method of doing this. 
    //if not we should write our own implementation that somehow speeds this operation
    uint clusterID = FindClusterIDFromClusters(cluster);
    cout << "adding label " << m_model.GetAnnotationClass() << " for cluster: " << clusterID << endl;
    AddLabel2ClusterMapping(m_model.GetAnnotationClass(),clusterID);
}
void BNSegmentator::ResegmentPointCluster(pcl::PointXYZRGB inPoint)
{
    pcl::PointIndices cluster;
    std::vector<int> indices1 (1);
    std::vector<float> distances (1);
        
    
    m_searchKDTree.nearestKSearch (inPoint, 1, indices1, distances);
    m_regionGrowingSegmentator.getSegmentFromPoint(indices1[0],cluster);

    //TODO do min cut segmentation here?


    //Siddhant: I am not sure if PCL has a more efficient method of doing this. 
    //if not we should write our own implementation that somehow speeds this operation
    uint clusterID = FindClusterIDFromClusters(cluster);
    cout << "adding label " << m_model.GetAnnotationClass() << " for cluster: " << clusterID << endl;
    AddLabel2ClusterMapping(m_model.GetAnnotationClass(),clusterID);
}
void BNSegmentator::UpdateLabelledPointCloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr labelledCloud = m_model.GetLabelledPointCloud();
    std::unordered_map<uint,std::vector<uint>>::iterator startIt = m_label2ClusterMap.begin();
    std::unordered_map<uint,std::vector<uint>>::iterator endIt = m_label2ClusterMap.end();

    while(startIt != endIt)
    {
        std::vector<uint>::iterator clusterStartIt = startIt->second.begin();
        std::vector<uint>::iterator clusterEndIt = startIt->second.end();

        while(clusterStartIt!=clusterEndIt)
        {
            pcl::PointIndices clusterPoints = m_clusters[*clusterStartIt];

            for(int i=0;i<clusterPoints.indices.size();i++)
            {
                BNLabelColor labelColor = m_labelStore.GetColorForLabel(startIt->first);
                labelledCloud->points[clusterPoints.indices[i]].r = labelColor.red;
                labelledCloud->points[clusterPoints.indices[i]].g = labelColor.green;
                labelledCloud->points[clusterPoints.indices[i]].b = labelColor.blue;
            }
            clusterStartIt ++;

        }
        startIt++;
    }


}