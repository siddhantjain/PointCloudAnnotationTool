#include "includes/BNModel.h"

BNModel::BNModel(BNState& inState, BNLabelStore& inStore,CONFIG& inConfig):
m_state(inState),
m_labelStore(inStore),
m_config(inConfig)
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
    isHumanAnnotated.resize(m_labelledPointCloud->points.size(),false);
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
int BNModel::GetAnnotationClass()
{
    return m_annotationClassNum;
}
void BNModel::SetAnnotationClass(int inClass)
{
    cout << "setting annotation class as: " << inClass << endl;
    m_annotationClassNum = inClass;
}
BNLabelStore& BNModel::GetLabelStore()
{
    return m_labelStore;
}

void BNModel::WriteLabelledPointCloud()
{
    cout << "Writing Labelled Point Cloud to File" << endl;
    std::ofstream pointCloudFile(m_config["OutputFileName"]);

    //Siddhant: Getting existing number of parts to start new labelling from this number
    int lastPartLabel = stoi(m_config["LastPartLabel"]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr labelledCloud = m_labelledPointCloud;

    int numPointsWritten = 0;

    for(int i=0;i<labelledCloud->points.size();i++)
    {
            float ptX = labelledCloud->points[i].x;
            float ptY = labelledCloud->points[i].y;
            float ptZ = labelledCloud->points[i].z;
            int label = m_labelStore.GetLabelForColor(labelledCloud->points[i].r,labelledCloud->points[i].g,labelledCloud->points[i].b);
            label = label-1 + lastPartLabel; 
            if(!isHumanAnnotated[i])
                label = -1;
            pointCloudFile << ptX << " " << ptY << " " << ptZ << " " << label << endl;
            numPointsWritten++;
    }
    cout << "numPointsWritten: " << numPointsWritten << endl;
}

void BNModel::CallPythonFineTune()
{
    std::string init_command = "chmod u+x ../src/Python/src/FineTune.py"; 
    system(init_command.c_str());
    std::string command = "python ../src/Python/src/FineTune.py";
    std::string arg_point_cloud = " --point_cloud_file " + m_config["OutputFileName"];
    std::string arg_num_classes = " --num_classes " + m_config["NumClasses"];
    std::string arg_model_path = " --model_path /Users/sowmya/Desktop/Capstone/code/github/main/PointCloudAnnotationTool/src/Python/trained_models/epoch_80.ckpt";
    std::string arg_output_dir = " --output_dir /Users/sowmya/Desktop/Capstone/code/github/main/PointCloudAnnotationTool/output";
    std::string arg_num_epochs = " --epoch 80";
    command += arg_point_cloud + arg_num_classes + arg_model_path + arg_output_dir + arg_num_epochs;
    system(command.c_str());
}
void BNModel::GetNewLabels()
{
    cout << "Reading new labels" << endl;
    int numPointsRead = 0;
    std::ifstream pointCloudFile(m_config["PythonOutputFileName"]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr labelledCloud = m_labelledPointCloud;
    for(int i=0;i<labelledCloud->points.size();i++)
    {
            float ptX,ptY,ptZ;
            float ptRed,ptBlue,ptGreen;
            pointCloudFile >> ptX >>  ptY >> ptZ >> ptRed >> ptGreen >> ptBlue;
            labelledCloud->points[i].x = ptX;
            labelledCloud->points[i].y = ptY;
            labelledCloud->points[i].z = ptZ;
            labelledCloud->points[i].r = ptRed;
            labelledCloud->points[i].g = ptGreen;
            labelledCloud->points[i].b = ptBlue;
            cout << " " << ptX << " " << ptY << " " << ptZ << " " << ptRed << endl ;
            numPointsRead++;
    }
    cout << "Read: " << numPointsRead << " point " << endl;
}