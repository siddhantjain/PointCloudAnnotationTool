#include "includes/BNPainter.h"
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

void crashHandler2(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

BNPainter::BNPainter(BNModel& inModel, BNLabelStore& inStore, CONFIG& prg_config):
m_model(inModel),
m_labelStore(inStore),
m_prgConfig(prg_config)
{
    cout << "Constructing Painter" << endl;
    InitPainter();
    m_brushSize = 5;
    
}

void BNPainter::InitPainter()
{
    signal(SIGSEGV, crashHandler2); 
    signal(SIGABRT, crashHandler2);
    pcl::IndicesPtr m_origCloudIndices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    
    m_searchKDTree.setInputCloud(m_model.GetRawPointCloud());

    //siddhant: This way of getting indices is pretty stupid. Come up with a better way
    pass.setInputCloud (m_model.GetRawPointCloud());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1000.0, 10000.0);
    pass.filter (*m_origCloudIndices);

    
    cout << "Painter Initialisation done" << endl;
}

void BNPainter::PaintNNeighbours(pcl::PointXYZRGB inPoint)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr labelledCloud = m_model.GetLabelledPointCloud();

    pcl::PointIndices cluster;
    std::vector<int> indices(m_brushSize);
    std::vector<float> distances(m_brushSize);
        

    m_searchKDTree.nearestKSearch (inPoint, m_brushSize, indices, distances);
    
    for (int i=0;i<indices.size();i++)
    {
                BNLabelColor labelColor = m_labelStore.GetColorForLabel(m_model.GetAnnotationClass());
                labelledCloud->points[indices[i]].r = labelColor.red;
                labelledCloud->points[indices[i]].g = labelColor.green;
                labelledCloud->points[indices[i]].b = labelColor.blue;
                m_model.isHumanAnnotated[indices[i]] = true;
    }
}

uint64_t BNPainter::GetBrushSize()
{
    return m_brushSize;
}

void BNPainter::SetBrushSize(uint64_t inBrushSize)
{
    m_brushSize = inBrushSize;
}