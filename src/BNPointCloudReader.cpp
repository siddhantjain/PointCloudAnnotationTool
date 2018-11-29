#include "includes/BNPointCloudReader.h"

BNPointCloudReader::BNPointCloudReader(std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudRef):
m_pointCloudFilePath(fileName),
m_pointCloudRef(pointCloudRef)
{
    ReadPointCloud();
}

std::string GetFileExtension(const std::string& filePath)
{    
    if(filePath.find_last_of(".") != std::string::npos)
        return filePath.substr(filePath.find_last_of(".")+1);
    return "";
}

void ConvertXYZToXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzPC,pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgbPC)
{
    cout << "Converting XYZ Point Cloud to XYZRGB Point Cloud" << endl;
    copyPointCloud(*xyzPC, *xyzrgbPC);
}

void BNPointCloudReader::SetFileName(std::string filePath)
{
    m_pointCloudFilePath = filePath;
}

void BNPointCloudReader::ReadPointCloud()
{
    std::string fileExtension = GetFileExtension(m_pointCloudFilePath);
    if(fileExtension=="pcd")
    {
        ReadPCDPointCloud();
    }
    else if(fileExtension=="txt")
    {
        ReadTxtPointCloud(false);
    }
    else if (fileExtension=="ply")
    {
        ReadPlyPointCloud();
    }
    else
    {
        std::cout << "Unidentified point cloud extension: " << fileExtension << " reading failed" << std::endl;
    }
}

void BNPointCloudReader::ReadPlyPointCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempXYZCloud (new pcl::PointCloud<pcl::PointXYZ>);
    if ( pcl::io::loadPLYFile <pcl::PointXYZ> (m_pointCloudFilePath, *tempXYZCloud) == -1)
        { 
            std::cout << "Cloud reading failed." << std::endl;
            return;
        }
    ConvertXYZToXYZRGB(tempXYZCloud,m_pointCloudRef);   
}
void BNPointCloudReader::ReadPCDPointCloud()
{
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (m_pointCloudFilePath, *m_pointCloudRef) == -1)
        { 
            std::cout << "Cloud reading failed." << std::endl;
            return;
        }
}

void BNPointCloudReader::ReadTxtPointCloud(bool isXYZRGB)
{
    //siddhant: current assumption is that all txt point clouds have no color info. Will change this later
    //to include whether pc file has color info or not in the config file and incorporate this as a feature at the class
    //level.
    pcl::ASCIIReader ptsReader;
    if(!isXYZRGB)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempXYZCloud (new pcl::PointCloud<pcl::PointXYZ>);
        ptsReader.setSepChars(" ");
        ptsReader.read(m_pointCloudFilePath,*tempXYZCloud);
        ConvertXYZToXYZRGB(tempXYZCloud,m_pointCloudRef);        
    }
    else
    {
        std::string line;
        std::ifstream pointCloudFile(m_pointCloudFilePath);
        int counter = 0;
        while(std::getline(pointCloudFile, line))
        {
            counter++;
            double ptX,ptY,ptZ;
            double ptRed,ptBlue,ptGreen;
            std::istringstream iss(line);
            iss >> ptX >>  ptY >> ptZ >> ptRed >> ptGreen >> ptBlue;
            pcl::PointXYZRGB thisPoint;
            thisPoint.r = uint8_t(ptRed);
            thisPoint.g = uint8_t(ptGreen);
            thisPoint.b = uint8_t(ptBlue);
            thisPoint.x = ptX;
            thisPoint.y = ptY;
            thisPoint.z = ptZ;
            m_pointCloudRef->push_back(thisPoint);
        }
        std::cout << "Points read: " << counter << std::endl;
    }

}