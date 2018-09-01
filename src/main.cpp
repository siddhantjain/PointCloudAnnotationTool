//New Includes

#include "includes/BNModel.h"
#include "includes/BNView.h"
#include "includes/BNSegmentator.h"
#include "includes/BNState.h"
#include "includes/BNConfigReader.h"

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

void ConvertXYZToXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzPC,pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgbPC)
{
    cout << "Converting XYZ Point Cloud to XYZRGB Point Cloud" << endl;
    copyPointCloud(*xyzPC, *xyzrgbPC);
}
bool InitModel(BNModel& inModel, std::string PCFileName, bool isXYZPointCloud)
{
    //std::string PCFileName = "../data/93.pcd";
    cout << "Initialising Model" << endl;
    cout << "Loading point cloud: " << PCFileName << endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    if(isXYZPointCloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr inXYZCloud (new pcl::PointCloud<pcl::PointXYZ>);  
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        if ( pcl::io::loadPCDFile <pcl::PointXYZ> (PCFileName, *inXYZCloud) == -1)
        { 
            std::cout << "Cloud reading failed." << std::endl;
            return false;
        }
        std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        std::cout << "Time taken to load = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

        ConvertXYZToXYZRGB(inXYZCloud,inCloud);
    }
    
    else
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (PCFileName, *inCloud) == -1)
        { 
            std::cout << "Cloud reading failed." << std::endl;
            return false;
        }
        std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        std::cout << "Time taken to load = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;
    }
      

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

