
#ifndef COMMON_H
#define COMMON_H

/*//C++ headers
#include <iostream>
#include <vector>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

//boost header
#include <boost/thread/thread.hpp>
*/
//PCL headers:
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/extract_clusters.h>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>



namespace BNUtils
{
    enum loglevel_e
    {logERROR, logWARNING, logINFO, logDEBUG, logDEBUG1, logDEBUG2, logDEBUG3, logDEBUG4};

    class BNLogger
    {
    public:
        BNLogger(loglevel_e _loglevel = logERROR) {
            _buffer << _loglevel << " :" 
                << std::string(
                    _loglevel > logDEBUG 
                    ? (_loglevel - logDEBUG) * 4 
                    : 1
                    , ' ');
        }

        template <typename T>
        BNLogger & operator<<(T const & value)
        {
            _buffer << value;
            return *this;
        }

        ~BNLogger()
        {
            _buffer << std::endl;
            // This is atomic according to the POSIX standard
            // http://www.gnu.org/s/libc/manual/html_node/Streams-and-Threads.html
            std::cerr << _buffer.str();
        }

    private:
        std::ostringstream _buffer;
    };

    //extern loglevel_e loglevel;

   // #define log(level) if (level > loglevel) ; else BNLogger(level)
}

#endif