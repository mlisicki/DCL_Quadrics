/*!
 * Quadrics
 * Michal Lisicki
 */

#ifndef QUADRICS_HPP_
#define QUADRICS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "Panel_Empty.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include <unordered_map>
#include <deque>
#include <cv.h>

namespace Processors {
namespace Quadrics {

struct hashP3 {
    size_t operator()(const cv::Point3_<uchar> &p) {
        return hash<int>()((int)p.x) ^ hash<int>()((int)p.y) ^ hash<int>()((int)p.z);
    }
};
    
typedef std::unordered_map< cv::Point3_<uchar>, std::deque<cv::Point2i>, hashP3> HMap;    
    
/*!
 * \class Quadrics
 * \brief Quadrics processor class.
 */
class Quadrics: public Base::Component
{
public:
	/*!
	 * Constructor.
	 */
	Quadrics(const std::string & name = "");

	/*!
	 * Destructor
	 */
	virtual ~Quadrics();


protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Retrieves data from device.
	 */
	bool onStep();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

        void onNewImage();
        void onNewPointCloud();
   
        Base::Property<int> paramLabelSize;
        Base::Property<int> paramInliersSize;
        Base::Property<float> paramTDistance;
        
	Base::EventHandler <Quadrics> h_onNewImage;
        Base::EventHandler <Quadrics> h_onNewPointCloud;
        
        // Labeled image
        Base::DataStreamIn <cv::Mat> inImg;
        // Point cloud
	Base::DataStreamIn <cv::Mat> inPC;
        
        
        Base::Event* newImage;
        
        // Data fitted by models
        Base::DataStreamOut <cv::Mat> outImg;
        // Labels as text data
        // Base::DataStreamOut <std::queue<SymbolicObject> > outData;
        
        cv::Mat segments;
        
        cv::Mat imageFrame;
        cv::Mat pcFrame;
        
        bool imageFrameReady;
	bool pcFrameReady;
        
        HMap pcClusters;
        
        static const int INVALID_PIXEL = 2047;
};

}//: namespace Quadrics
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_PROCESSOR_COMPONENT("Quadrics", Processors::Quadrics::Quadrics, Common::Panel_Empty)

#endif /* QUADRICS_HPP_ */
