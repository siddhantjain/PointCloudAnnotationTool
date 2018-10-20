
#include "BNModel.h"
#include "BNSegmentator.h"
#include "BNPainter.h"
class BNView
{
public:
    BNView(BNModel& inModel, BNSegmentator& inSegmentator, BNPainter& inPainter);
    void InitView();
    void VisualiseLabelledCloud();
    void VisualiseRawCloud();
    void VisualiseSegmentedPointCloud();
    void InvokeSettingsPanel();
private:
    void RegisterHandlers();
    void KeyboardEventHandler(const pcl::visualization::KeyboardEvent &event, void* cookie);
    void PointPickingCallbackEventHandler(const pcl::visualization::PointPickingEvent& event, void* cookie);
    void AreaPickingEventHandler(const pcl::visualization::AreaPickingEvent& event, void* cookie);
    void RefreshStateView();
    void AnnotationCLI();
    void AnnotationModeKeyEventHandler(const pcl::visualization::KeyboardEvent &event);
    BNModel& m_model;
    BNSegmentator& m_segmentator;
    BNPainter& m_painter;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
    void WritePointCloudToFile();
    void GetNewLabelsKeyPressed();
};