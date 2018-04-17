
#include "BNModel.h"

class BNView
{
public:
    BNView(BNModel& inModel);
    void InitView();
    void VisualiseLabelledCloud();
    void VisualiseRawCloud();
    void VisualiseSegmentedPointCloud();    
private:
    void (keyboardCallBack)(const pcl::visualization::KeyboardEvent &event, void* cookie);
    void RegisterHandlers();
    void KeyboardEventHandler(const pcl::visualization::KeyboardEvent &event, void* cookie);
    void PointPickingCallbackEventHandler(const pcl::visualization::PointPickingEvent& event, void* cookie);
    void RefreshStateView();
    void AnnotationCLI();
    BNModel& m_model;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
};