#include "common.h"
#include "BNModel.h"
#include "BNLabelStore.h"
#include <pcl/search/search.h>

#ifndef BN_PAINTER_H
#define BN_PAINTER_H


class BNPainter
{
public:
    BNPainter(BNModel& inModel, BNLabelStore& inLabelStore, CONFIG& inConfig);
    void PaintNNeighbours(pcl::PointXYZRGB inPoint);
    void SetBrushSize(uint64_t inBrushSize);
    uint64_t GetBrushSize();
private:
	void InitPainter();
    uint64_t m_brushSize;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloud;
    BNModel& m_model;
    BNLabelStore& m_labelStore;
    pcl::search::KdTree<pcl::PointXYZRGB> m_searchKDTree;
    pcl::IndicesPtr m_origCloudIndices;
    CONFIG& m_prgConfig;
};

#endif