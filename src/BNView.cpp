#include "includes/BNView.h"
#include <functional>

BNView::BNView(BNModel& inModel):
m_model(inModel)
{
    std::cout << "View Created" << std::endl;
}

void BNView::KeyboardEventHandler(const pcl::visualization::KeyboardEvent &event, void* cookie)
{
    cout << "Keyboard event occurred" << endl;
    //BNView::VisualiseLabelledCloud();
    VisualiseSegmentedPointCloud();
}

void BNView::PointPickingCallbackEventHandler(const pcl::visualization::PointPickingEvent& event, void* cookie)
{
    cout << "Point Picking Event Occured" << endl;
  //  VisualiseRawCloud();
}

void BNView::RegisterHandlers()
{
    
    //auto fp = std::bind(&BNView::KeyboardEventHandler, NULL);
    m_viewer->registerKeyboardCallback(&BNView::KeyboardEventHandler,*this,(void*)NULL);
    m_viewer->registerPointPickingCallback(&BNView::PointPickingCallbackEventHandler,*this,(void*)NULL);
}
void BNView::VisualiseLabelledCloud()
{
    m_viewer->removeAllPointClouds();
    m_viewer->addPointCloud<pcl::PointXYZRGB> (m_model.GetLabelledPointCloud(), "Labelled Point Cloud");
    //siddhant: do we need this?
    m_viewer->spinOnce (1);
}
void BNView::VisualiseRawCloud()
{
    m_viewer->removeAllPointClouds();
    m_viewer->addPointCloud<pcl::PointXYZRGB> (m_model.GetRawPointCloud(), "Raw Point Cloud");
    //siddhant: do we need this?
    m_viewer->spinOnce (1);
}
void BNView::VisualiseSegmentedPointCloud()
{
    m_viewer->removeAllPointClouds();
    cout << "Removal Succesful" << endl;
    m_viewer->addPointCloud<pcl::PointXYZRGB> (m_model.GetSegmentedPointCloud(), "Segmented Point Cloud");
    //siddhant: do we need this?
    m_viewer->spinOnce (1);   
}
void BNView::InitView() 
{
    cout << "Creating a PCL Visualiser for the view" << endl;
     //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    m_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("BN PCL Viewer"));

    m_viewer->setBackgroundColor (0, 0, 0);
    m_viewer->addPointCloud<pcl::PointXYZRGB> (m_model.GetRawPointCloud(), "Raw Point Cloud");
    m_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Raw Point Cloud");
    m_viewer->initCameraParameters ();

    RegisterHandlers();

    while (!m_viewer->wasStopped ())
    {
        m_viewer->spinOnce (1);
    }


}