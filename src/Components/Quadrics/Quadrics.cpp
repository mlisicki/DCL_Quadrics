/*!
 * Quadrics
 * Michal Lisicki
 */

#include <memory>
#include <string>

#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <eigen2/Eigen/src/Core/MatrixBase.h>

#include "Quadrics.hpp"
#include "Common/Logger.hpp"

namespace Processors {
namespace Quadrics {

Quadrics::Quadrics(const std::string & name) : 
        Base::Component(name),
        paramLabelSize("label_size",500),
        paramInliersSize("inliers_size",100),
        paramTDistance("t_distance",0.0001f) {
    
        LOG(LTRACE) << "Hello Quadrics\n";
	registerProperty(paramLabelSize);
	registerProperty(paramInliersSize);
        registerProperty(paramTDistance);
}

Quadrics::~Quadrics()
{
	LOG(LTRACE) << "Good bye Quadrics\n";
}

bool Quadrics::onInit()
{
	LOG(LTRACE) << "Quadrics::initialize\n";
	
        h_onNewPointCloud.setup(this, &Quadrics::onNewPointCloud);
	registerHandler("onNewPointCloud", &h_onNewPointCloud);
	h_onNewImage.setup(this, &Quadrics::onNewImage);
	registerHandler("onNewImage", &h_onNewImage);

	registerStream("in_pc", &inPC);
        registerStream("in_img", &inImg);
        
        // Register data streams
        registerStream("out_img", &outImg);

        newImage = registerEvent("newImage");
        
        imageFrame = cv::Mat( cv::Size(640,480),CV_8UC3,cv::Scalar(0) );
        pcFrame = cv::Mat( 640, 480, CV_32FC3, cv::Scalar::all(INVALID_PIXEL) );

	return true;
}

bool Quadrics::onFinish()
{
	LOG(LTRACE) << "Quadrics::finish\n";

	return true;
}

bool Quadrics::onStep()
{
	LOG(LTRACE) << "Quadrics::step\n";
        try {
            segments = cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar::all(0));
            pcClusters.clear();
            
            for(int x=0; x<640; x++) {
                for(int y=0; y<480; y++) {
                    if(imageFrame.at<cv::Point3_<uchar> >(y,x)!=cv::Point3_<uchar>(0,0,0)) {
                        pcClusters[imageFrame.at<cv::Point3_<uchar> >(y,x)].push_back(cv::Point2i(y,x));
                    }
                }
            }
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            HMap::const_iterator it;
           // std::dequeue<cv::Point2i> >::const_iterator dqit;
            for ( it=pcClusters.begin() ; it != pcClusters.end(); ++it ) {
                // initialize PointClouds
                // pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
                
                // populate our PointCloud with points
                cloud->width = (it->second).size();
                // std::cout << cloud->width << std::endl;
                cloud->height   = 1;
                cloud->is_dense = false;
                cloud->points.resize(cloud->width * cloud->height);
                
                for (int i=0; i < cloud->points.size(); i++) {
                    cloud->points[i].x = pcFrame.at<cv::Point3f>((it->second).at(i).x, (it->second).at(i).y).x;
                    cloud->points[i].y = pcFrame.at<cv::Point3f>((it->second).at(i).x, (it->second).at(i).y).y;
                    cloud->points[i].z = pcFrame.at<cv::Point3f>((it->second).at(i).x, (it->second).at(i).y).z;
                }
                if(cloud->points.size() > paramLabelSize) {
                    std::vector<int> inliers;      

                    // created RandomSampleConsensus object and compute the appropriated model
                    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

                    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
                    ransac.setDistanceThreshold(paramTDistance);
                    ransac.computeModel();
                    ransac.getInliers(inliers);

                    if(inliers.size() > paramInliersSize) {
                        cv::Point3f sum(0,0,0);
                        cv::Point3f min(0,0,0);
                        cv::Point3f max(0,0,0);
                        
                        for (int i=0; i < inliers.size(); i++) {
                            if(i==0) {
                                min = pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y);
                                max = min;
                            }
                            // Find bounding box parameters
                            if(pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y).x < min.x) min.x = pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y).x;
                            if(pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y).y < min.y) min.y = pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y).y;
                            if(pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y).z < min.z) min.z = pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y).z;
                            if(pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y).x > max.x) max.x = pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y).x;
                            if(pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y).y > max.y) max.y = pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y).y;
                            if(pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y).z > max.z) max.z = pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y).z;

                            segments.at<cv::Point3_<uchar> >((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y) = (it->first);
                            sum += pcFrame.at<cv::Point3f>((it->second).at(inliers[i]).x, (it->second).at(inliers[i]).y);
                        }
                        sum.x *= sum.x/inliers.size();
                        sum.y *= sum.y/inliers.size();
                        sum.z *= sum.z/inliers.size();
                        
                        Eigen::VectorXf modelCoefficients;
                        
                        ransac.getModelCoefficients(modelCoefficients);
                        
                        std::cout << "face(" << sum  << "," << std::endl;
                        std::cout << "     [";
                        for(int i=0; i<modelCoefficients.size()-1; i++) std::cout << modelCoefficients[i] << ", ";
                        std::cout << modelCoefficients[modelCoefficients.size()-1] << "]," << std::endl;
                        std::cout << "     [";
                        std::cout << min.x << ", " << min.y << ", " << min.z << ", " << max.x << ", " << max.y << ", " << max.z << "])." << std::endl;
                    }
                }
                cloud->clear();
            }
            std::cout << std::endl << std::endl;
            


            // pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

//            for (size_t i = 0; i < final->points.size (); ++i) {
////                if(final->points[i].y >= 0 && final->points[i].y < 480 &&
////                    final->points[i].x >= 0 && final->points[i].x < 640)
//                    segments.at<float>((int)(final->points[i].y), (int)(final->points[i].x)) = (float)(final->points[i].z);
//            }
//            
//            // Calculate center of mass as position
//            pcClusters[0].size();
//            cv::Point3i sum(0,0,0);
//            
//            for ( it=headers_.begin() ; it != headers_.end(); ++it ) {
//                int pcSize = (it->second)->size();
//                for (int i=0; i < pcSize; i++) {
//                    sum += cv::Point3i((it->second)->at(i).x, (it->second)->at(i).y, pcFrame.at<float>((it->second)->at(i).x, (it->second)->at(i).y));
//                }
//                cv::Point3f centerOfMass = sum / (it->second)->size();
//                
//                ransac.getModelCoefficients(Eigen::VectorXf& model_coefficients); 
//            }

            outImg.write(segments.clone());
            newImage->raise();
        } catch (...) {
            LOG(LERROR) << "Quadrics::onStep failed\n";
        }
        return true;
}

bool Quadrics::onStop()
{
	return true;
}

bool Quadrics::onStart()
{
	return true;
}

void Quadrics::onNewImage() {
	LOG(LTRACE) << "New Image";
	imageFrame = inImg.read().clone();
	imageFrameReady = true;

	if (pcFrameReady && imageFrameReady)
		onStep();
}

void Quadrics::onNewPointCloud() {
	LOG(LTRACE) << "New Point Cloud";
	pcFrame = inPC.read().clone();
	pcFrameReady = true;

	if (pcFrameReady && imageFrameReady)
		onStep();
}

}//: namespace Quadrics
}//: namespace Processors
