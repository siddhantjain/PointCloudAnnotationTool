//New Includes

#include "includes/BNModel.h"
#include "includes/BNView.h"
#include "includes/BNSegmentator.h"
#include "includes/BNState.h"
//New includes end

#include "includes/common.h"


bool InitSegmentator(BNSegmentator& inSegmentator, BNModel& inModel)
{
    //do nothing
}

bool InitView(BNView& inView, BNModel& inModel)
{
    inView.InitView();
    return true;
}

bool InitModel(BNModel& inModel, std::string PCFileName)
{
    //std::string PCFileName = "../data/93.pcd";
    cout << "Initialising Model" << endl;
    cout << "Loading point cloud: " << PCFileName << endl;

    //siddhant: we are allowing only RGB point clouds. Templatise this?
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (PCFileName, *inCloud) == -1)
    { 
        std::cout << "Cloud reading failed." << std::endl;
        return false;
    }
    std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
    std::cout << "Time taken to load = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

    inModel.InitModel(inCloud);
    return true;
}


int main (int argc, char** argv)
{
    cout << "Welcome to our point cloud annotation tool. As of now, a new window will open where you can manipulate the point cloud. All messages will be shown in this video" << endl;
    BNLabelStore toolLabelStore;
    BNState toolStateMachine;
    BNModel toolModel(toolStateMachine,toolLabelStore);

    if(!InitModel(toolModel,"../data/learn17.pcd"))
    {
      return -1;
    }

    cout << "Segmenting the point cloud" << endl;
    BNSegmentator toolSegmentator(toolModel,toolLabelStore);
    
    BNView toolView(toolModel,toolSegmentator);
    if(!InitView(toolView,toolModel))
    {
      return -1;
    }


    
}

