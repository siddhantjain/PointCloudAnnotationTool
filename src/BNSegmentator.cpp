#include "includes/BNSegmentator.h"

#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

void crashHandler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

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
    signal(SIGSEGV, crashHandler); 
    signal(SIGABRT, crashHandler);
    //siddhant: Check if we really need this indices business? 
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::IndicesPtr m_origCloudIndices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (m_model.GetRawPointCloud());
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    //siddhant: we have two trees in the segmentator right now
    // one of them is mostly redundant. Figure out which one and resolve this stupidity
    m_searchKDTree.setInputCloud(m_model.GetRawPointCloud());

    pass.setInputCloud (m_model.GetRawPointCloud());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 10.0);
    pass.filter (*m_origCloudIndices);

    m_regionGrowingSegmentatorRGB.setInputCloud (m_model.GetRawPointCloud());
    m_regionGrowingSegmentatorRGB.setIndices (m_origCloudIndices);
    m_regionGrowingSegmentatorRGB.setSearchMethod (tree);
    m_regionGrowingSegmentatorRGB.setDistanceThreshold (10);
    m_regionGrowingSegmentatorRGB.setPointColorThreshold (4);
    m_regionGrowingSegmentatorRGB.setRegionColorThreshold (1);
    m_regionGrowingSegmentatorRGB.setMinClusterSize (600); 

  

    
    m_regionGrowingSegmentatorN.setMinClusterSize (600);
    m_regionGrowingSegmentatorN.setMaxClusterSize (1000000);
    m_regionGrowingSegmentatorN.setSearchMethod (tree);
    m_regionGrowingSegmentatorN.setNumberOfNeighbours (24);
    m_regionGrowingSegmentatorN.setInputCloud (m_model.GetRawPointCloud());
      //reg.setIndices (indices);
    m_regionGrowingSegmentatorN.setInputNormals (normals);
    m_regionGrowingSegmentatorN.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    m_regionGrowingSegmentatorN.setCurvatureThreshold (1.0);


    m_minCutSegmentator.setInputCloud (m_model.GetRawPointCloud());
    m_minCutSegmentator.setSigma (0.15);
    m_minCutSegmentator.setRadius (3);
    m_minCutSegmentator.setNumberOfNeighbours (8);
    m_minCutSegmentator.setSourceWeight (0.05);

    
    m_euclideanSegmentator.setInputCloud(m_model.GetRawPointCloud());
    m_euclideanSegmentator.setClusterTolerance (0.002); // 2cm
    m_euclideanSegmentator.setMinClusterSize (10);
    m_euclideanSegmentator.setMaxClusterSize (250000);
    m_euclideanSegmentator.setSearchMethod (tree);
    




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
    DoColorRegionGrowingSegmentation();
    //DoNormalRegionGrowingSegmentation();
}

void BNSegmentator::DoColorRegionGrowingSegmentation()
{
    cout << "Doing color based region growing segmentation" << endl;
    m_regionGrowingSegmentatorRGB.extract(m_clusters);
    m_model.SetSegmentedPointCloud(m_regionGrowingSegmentatorRGB.getColoredCloud());
}

void BNSegmentator::DoNormalRegionGrowingSegmentation()
{
    cout << "Doing normal region growing segmentation" << endl;
    m_regionGrowingSegmentatorN.extract(m_clusters);
    m_model.SetSegmentedPointCloud(m_regionGrowingSegmentatorN.getColoredCloud());
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

uint BNSegmentator::FindClusterIDFromPointIndex(int inPointIndex)
{
    cout << "Finding point in cluster of size: " << m_clusters.size() << endl;
    std::vector<pcl::PointIndices>::iterator startIt = m_clusters.begin();
    std::vector <pcl::PointIndices>::iterator endIt = m_clusters.end();

    uint clusterID=0;
    while(startIt!=endIt)
    {
        cout << clusterID << ": cluster size->" << startIt->indices.size() << endl; 
        if(find(startIt->indices.begin(), startIt->indices.end(), inPointIndex) != startIt->indices.end())
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
    
    //m_regionGrowingSegmentatorRGB.getSegmentFromPoint(indices1[0],cluster);

    uint clusterID = FindClusterIDFromPointIndex(indices1[0]);
    cout << "adding label " << m_model.GetAnnotationClass() << " for cluster: " << clusterID << endl;
    AddLabel2ClusterMapping(m_model.GetAnnotationClass(),clusterID);
}
void BNSegmentator::ResegmentPointCluster(pcl::PointXYZRGB inPoint,uint mode)
{
    pcl::PointIndices cluster;

    std::vector <pcl::PointIndices> newClusters;
    std::vector<int> indices1 (1);
    std::vector<float> distances (1);
        
    
    m_searchKDTree.nearestKSearch (inPoint, 1, indices1, distances);
    

    uint origClusterID = FindClusterIDFromPointIndex(indices1[0]);
    cout << "Cluster Search Successful: " << endl;
    cout << "origClusterID: " << origClusterID << endl;

    if(origClusterID >= m_clusters.size())
    {
        cout << "resegmenting a cluster which doesn't exist. Returning" << endl;
        return; 
    }

    pcl::IndicesPtr clusterIndices (new std::vector <int>);
    clusterIndices->insert(clusterIndices->end(),m_clusters[origClusterID].indices.begin(),m_clusters[origClusterID].indices.end());
 
    if(mode == 0)   
    {
        m_regionGrowingSegmentatorN.setIndices(clusterIndices);
        m_regionGrowingSegmentatorN.extract(newClusters);
        m_regionGrowingSegmentatorN.setIndices(m_origCloudIndices);
        cout << "Normal based Region growing extraction succesful. " << newClusters.size() << " clusters found" << endl;
    }
    else
    {
        m_euclideanSegmentator.setIndices (clusterIndices);
        m_euclideanSegmentator.extract (newClusters);
        cout << "Euclidean clustering based Region growing succesful. " << newClusters.size() << " clusters found" << endl;
    }
    
     
    
    //replacing original cluster with first cluster
    //m_clusters[origClusterID].indices().clear(); 
    if(newClusters.size()==0)
    {
        cout << "No new clusters found" << endl;
        return;
    }
    m_clusters[origClusterID]= newClusters[0];
    cout << "inserting cluster with size: " << newClusters[0].indices.size() << endl;
    //AddLabel2ClusterMapping(0,origClusterID);
    //adding background cluster
    for(int i=1;i<newClusters.size();i++)
    {
        m_clusters.push_back(newClusters[i]);
        cout << "inserting cluster with size: " << newClusters[i].indices.size() << endl;
        AddLabel2ClusterMapping(0,m_clusters.size()-1);
    }

    pcl::PointIndices unmarkedPoints;
    std::unordered_set<int> allPoints;

    //siddhant: This seems super inefficient and is slowing things down. Think of a better way to do this
    for(int i=0;i<clusterIndices->size();i++)
    {
        allPoints.insert(clusterIndices.get()->at(i));

    }


    std::vector<pcl::PointIndices>::iterator startIt = newClusters.begin();
    std::vector <pcl::PointIndices>::iterator endIt = newClusters.end();

    uint clusterID=0;
    

    while(startIt!=endIt)
    {

        for(int j=0;j<startIt->indices.size();j++)
        {
            int Pt = startIt->indices[j];
            if(allPoints.find(Pt) != allPoints.end())
            {
                allPoints.erase ( Pt );   
            }
        }
        startIt++;
    }

    std::unordered_set<int>::iterator setIt;
    for(setIt=allPoints.begin(); setIt!=allPoints.end() ;setIt ++)
    {
            unmarkedPoints.indices.push_back(*setIt);
    }
    

    m_clusters.push_back(unmarkedPoints);
    AddLabel2ClusterMapping(0,m_clusters.size()-1);
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


void BNSegmentator::AutoCompleteLabelling()
{
    
}