/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "GreedyVerification.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace GreedyVerification {

GreedyVerification::GreedyVerification(const std::string & name) :
		Base::Component(name) , 
        resolution("resolution", 0.005f),
        inlier_treshold("inlier_treshold", 0.005f),
        lambda("lambda", 1.5f){
	registerProperty(resolution);
	registerProperty(inlier_treshold);
    registerProperty(lambda);

}

GreedyVerification::~GreedyVerification() {
}

void GreedyVerification::prepareInterface() {
	// Register data streams, events and event handlers HERE!
    registerStream("in_aligned_hypotheses", &in_aligned_hypotheses);
    registerStream("in_cloud_xyzsift_scene", &in_cloud_xyzsift_scene);
    registerStream("out_verified_hypotheses", &out_verified_hypotheses);
	// Register handlers
	registerHandler("verify", boost::bind(&GreedyVerification::verify, this));
    addDependency("verify", &in_aligned_hypotheses);
	addDependency("verify", &in_cloud_xyzsift_scene);

}

bool GreedyVerification::onInit() {

	return true;
}

bool GreedyVerification::onFinish() {
	return true;
}

bool GreedyVerification::onStop() {
	return true;
}

bool GreedyVerification::onStart() {
	return true;
}

void GreedyVerification::verify() {
    CLOG(LTRACE) << "GreedyVerification::verify";
    pcl::PointCloud<PointXYZSIFT>::Ptr scene = in_cloud_xyzsift_scene.read();
    std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> aligned_hypotheses = in_aligned_hypotheses.read();

    if(aligned_hypotheses.size() == 0){
        CLOG(LINFO) << "GreedyVerification No hypotheses available";
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*scene, *scene_xyz);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> aligned_hypotheses_xyz;
    for(int i = 0; i < aligned_hypotheses.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*(aligned_hypotheses[i]), *cloud );
        aligned_hypotheses_xyz.push_back(cloud);
    }

    pcl::GreedyVerification<pcl::PointXYZ, pcl::PointXYZ> greedy_hv(lambda);
    greedy_hv.setResolution (resolution);
    greedy_hv.setInlierThreshold (inlier_treshold);
    greedy_hv.setSceneCloud (scene_xyz);
    greedy_hv.addModels (aligned_hypotheses_xyz, true);
    greedy_hv.verify ();
    std::vector<bool> mask_hv;
    greedy_hv.getMask (mask_hv);

    if(mask_hv.size() != aligned_hypotheses.size()){
       CLOG(LERROR) << "GreedyVerification wrong vector size";
    }
    std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> verified_hypotheses;
    for(int i = 0; i < mask_hv.size(); i++){
        if(mask_hv[i]){
            verified_hypotheses.push_back(aligned_hypotheses[i]);
            CLOG(LINFO) << "GreedyVerification: Hypothese " << i << " is CORRECT";
        }
        else{
            CLOG(LINFO) << "GreedyVerification: Hypothese " << i << " is NOT correct";
        }

    }

    out_verified_hypotheses.write(verified_hypotheses);
}



} //: namespace GreedyVerification
} //: namespace Processors
