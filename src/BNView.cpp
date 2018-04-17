#include "includes/BNView.h"
#include <functional>

BNView::BNView(BNModel& inModel):
m_model(inModel)
{
    std::cout << "View Created" << std::endl;
}
void BNView::RefreshStateView()
{

    //siddhant: This is a sad shortcut. What we ideally want is the state machine to pass a message that viewer subscribes
    // this message will be published everytime state changes. Alas, ain't nobody got time for that
    if(!m_viewer->updateText(m_model.GetState(),10, 10,40,1.0,1.0,0,"StateText"))
    {
        cout << "Trying to add text" << endl;
        bool isSuccesful = m_viewer->addText(m_model.GetState(),10, 10, 40, 1.0,1.0,0,"StateText");   
        cout << "Adding text succesful? " << isSuccesful << endl;
    }      

}
void BNView::KeyboardEventHandler(const pcl::visualization::KeyboardEvent &event, void* cookie)
{
    cout << "Keyboard event occurred" << endl;
 
    if (event.getKeySym () == "a" && event.keyDown ())
    {   
        m_model.SetState("Annotate");
        VisualiseSegmentedPointCloud();
    }
    if (event.getKeySym () == "r" && event.keyDown ())
    {   
        m_model.SetState("Raw Point Cloud");
        VisualiseRawCloud();
    }
    if (event.getKeySym () == "c" && event.keyDown ())
    {   
        m_model.SetState("Current Labelled Cloud");
        VisualiseLabelledCloud();
    }

    RefreshStateView();
}

void BNView::PointPickingCallbackEventHandler(const pcl::visualization::PointPickingEvent& event, void* cookie)
{
    cout << "Point Picking Event Occured" << endl;
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
    RefreshStateView();

    while (!m_viewer->wasStopped ())
    {
        m_viewer->spinOnce (1);
    }
}

void BNView::AnnotationCLI()
{
    //siddhant: This can be a separate class I guess. Again, ain't nobody got time for good code in research?
    cout << "Welcome to the annotation interface. To annotate, type in a class name. All clusters you select will then be labelled as that class" << endl;
    cout << "When you are done, press q" << endl;

    //TODO: Siddhantto
    /*
        Run a while loop waiting for user input, unless input is q.
        deregister keyboard events (may not be trivial)
        Whatever user writes down as class name, associate cluster with that class name
        Eventually, we will figure out how to serialise the map and write down to JSON or something so that the deep learning
        code can take it up as an input 
    */
}