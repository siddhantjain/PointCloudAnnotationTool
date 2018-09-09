//New Includes

#include "includes/BNModel.h"
#include "includes/BNView.h"
#include "includes/BNSegmentator.h"
#include "includes/BNState.h"
#include "includes/BNConfigReader.h"
#include "includes/BNPointCloudReader.h"

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

bool InitModel(BNModel& inModel, std::string PCFileName, bool isXYZPointCloud)
{
    //std::string PCFileName = "../data/93.pcd";
    cout << "Initialising Model" << endl;
    cout << "Loading point cloud: " << PCFileName << endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    BNPointCloudReader pcReader(PCFileName,inCloud);
    inModel.InitModel(inCloud);
    return true;
}


int main (int argc, char** argv)
{
    BNUtils::loglevel_e loglevel = BNUtils::logERROR;
    loglevel = BNUtils::logINFO;

    BNUtils::BNLogger(BNUtils::logINFO) << "Welcome to our point cloud annotation tool. As of now, a new window will open where you can manipulate the point cloud. All messages will be shown in this window";
    
    BNLabelStore toolLabelStore;
    BNState toolStateMachine;
    BNModel toolModel(toolStateMachine,toolLabelStore);

    bool isXYZPointCloud = argc>1;

    BNConfigReader configReader("../src/cfg_file.txt");
    CONFIG prg_config = configReader.GetConfig();
    
    if(!InitModel(toolModel,prg_config["pointcloud"],isXYZPointCloud))
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

