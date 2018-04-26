#include "includes/BNView.h"
#include <functional>

BNView::BNView(BNModel& inModel, BNSegmentator& inSegmentator):
m_model(inModel),
m_segmentator(inSegmentator)
{
    std::cout << "View Created" << std::endl;
}

void DisplayAllLabelNames(boost::shared_ptr<pcl::visualization::PCLVisualizer> inViewer, BNLabelStore& inLabelStore)
{
    std::vector<BNLabel>& modelLabels = inLabelStore.GetLabels();

    for(int i=0;i<modelLabels.size();i++)
    {
        std::string ClassName = "Label: " + modelLabels[i].m_labelName + "( " + std::to_string(i) + " )";
        cout << "Displaying: " << ClassName << endl;
        BNLabelColor classColor = modelLabels[i].m_color;
        uint fontSize = 40;
        uint posX = 10;
        uint posY = 800 - i*(fontSize+5);
        if(!inViewer->updateText(ClassName,posX, posY,10,classColor.red/255.0,classColor.green/255.0,classColor.blue/255.0,ClassName))
        {
            
            inViewer->addText(ClassName,posX, posY, 10, classColor.red/255.0,classColor.green/255.0,classColor.blue/255.0,ClassName);   
        }
    }
}
void BNView::RefreshStateView()
{

    //siddhant: This is a sad shortcut. What we ideally want is the state machine to pass a message that viewer subscribes
    // this message will be published everytime state changes. Alas, ain't nobody got time for that
    
    //Also, the location of the labels should be dependent on the screen size (making this portable across screens will be fun, lol)

    

    if (m_model.GetState() == "Annotate" || m_model.GetState() == "Correct" )
    {
        if(!m_viewer->updateText(m_model.GetState(),10, 10,40,1.0,1.0,0,"StateText"))
        {
            std::string modeLabelText = "Mode: " + m_model.GetState();
            //  m_viewer->updateText("",1000, 10,40,0,0,0,"AnnotationClassText"); 
            m_viewer->addText(modeLabelText,10, 10, 40, 1.0,1.0,0,"StateText");   
        }

        DisplayAllLabelNames( m_viewer,m_model.GetLabelStore());
        std::string ClassName = "Class: " + m_model.GetLabelStore().GetNameForLabel(m_model.GetAnnotationClass());
        BNLabelColor classColor = m_model.GetLabelStore().GetColorForLabel(m_model.GetAnnotationClass());
        if(!m_viewer->updateText(ClassName,1000, 10,40,classColor.red/255.0,classColor.green/255.0,classColor.blue/255.0,"AnnotationClassText"))
        {
            
            m_viewer->addText(ClassName,1200, 10, 40, classColor.red/255.0,classColor.green/255.0,classColor.blue/255.0,"AnnotationClassText");   
        }
    } 
    else
    {
        if(!m_viewer->updateText(m_model.GetState(),10, 10,40,1.0,1.0,0,"StateText"))
        {
            std::string modeLabelText = "Mode: " + m_model.GetState();
            //  m_viewer->updateText("",1000, 10,40,0,0,0,"AnnotationClassText"); 
            m_viewer->addText(modeLabelText,10, 10, 40, 1.0,1.0,0,"StateText");   
        }
    }
 

}
void BNView::AnnotationModeKeyEventHandler(const pcl::visualization::KeyboardEvent &event)
{
    cout << "Annotation Key event handler, key pressed: " << event.getKeySym () << endl;
    //Siddhant: Haha. WTH is this code? Change it ASAP.
    if(event.getKeySym() >= std::to_string(0) && event.getKeySym() < std::to_string(m_model.GetLabelStore().GetLabels().size()))
    {
        m_model.SetAnnotationClass(stoi(event.getKeySym()));
    }
    
    if (event.getKeySym () == "c" )
    {   
        m_model.SetState("Resegment");
    }
    if (event.getKeySym () == "r" )
    {   
        m_model.SetState("Euclidean");
    }
    if (event.getKeySym () == "i" )
    {   
        m_model.SetState("Init");
        VisualiseRawCloud();
    }


    RefreshStateView();    
}
void BNView::KeyboardEventHandler(const pcl::visualization::KeyboardEvent &event, void* cookie)
{
    cout << "Keyboard event occurred" << endl;

    if (m_model.GetState() == "Annotate")
    {
        AnnotationModeKeyEventHandler(event);
        return;
    }

    if (event.getKeySym () == "a" && event.keyDown ())
    {   
        m_model.SetState("Annotate");
        VisualiseLabelledCloud();
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
    cout << "Point Picking Detected" << endl;
    pcl::PointXYZRGB picked_point;
    event.getPoint(picked_point.x, picked_point.y, picked_point.z);

    if (m_model.GetState() == "Annotate")
    {
        m_segmentator.AnnotatePointCluster(picked_point);
        cout << "Annotation of point cluster done" << endl;
        m_segmentator.UpdateLabelledPointCloud();
        cout << "segmented cloud updated" << endl;
        VisualiseLabelledCloud();
    }    
    if (m_model.GetState() == "Resegment")
    {
        m_segmentator.ResegmentPointCluster(picked_point,0);
        cout << "Resegmentation of point cluster done" << endl;
        m_segmentator.UpdateLabelledPointCloud();
        cout << "segmented cloud updated" << endl;
        VisualiseLabelledCloud();    
    }
    if (m_model.GetState() == "Euclidean")
    {
        m_segmentator.ResegmentPointCluster(picked_point,1);
        cout << "Resegmentation of point cluster done" << endl;
        m_segmentator.UpdateLabelledPointCloud();
        cout << "segmented cloud updated" << endl;
        VisualiseLabelledCloud();    
    }

}

void BNView::RegisterHandlers()
{
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

void ShowClasses(std::vector<BNLabel>& modelLabels)
{
    cout << "We have the following classes to label" << endl;
    for (int i=0;i<modelLabels.size();i++)
    {
        cout << i+1 << ": " << modelLabels[i].m_labelName << endl;
    }
}
void BNView::AnnotationCLI()
{
    //siddhant: This can be a separate class I guess. Again, ain't nobody got time for good code in research?
    cout << "Welcome to the annotation interface. To annotate, type in a class name. All clusters you select will then be labelled as that class" << endl;
    cout << "When you are done, press q" << endl;

    ShowClasses(m_model.GetLabelStore().GetLabels());

    std::string answer = "";

    while(answer != "quit")
    {
        cout << "command: " ;
        cin >> answer;
        cout << endl;

        if(answer == "list")
        {
            ShowClasses(m_model.GetLabelStore().GetLabels());
        }
        else if(answer == "quit")
        {
            break;
        }
    }
}
