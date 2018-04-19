
#include "BNModel.h"
#include "BNSegmentator.h"

class BNView
{
public:
    BNView(BNModel& inModel, BNSegmentator& inSegmentator);
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
    void AnnotationModeKeyEventHandler(const pcl::visualization::KeyboardEvent &event);
    BNModel& m_model;
    BNSegmentator& m_segmentator;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
};