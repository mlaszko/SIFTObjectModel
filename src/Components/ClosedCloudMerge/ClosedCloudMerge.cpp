
//
#include "ClosedCloudMerge.hpp"
#include "Common/Logger.hpp"

#include <Types/MergeUtils.hpp>
#include <boost/bind.hpp>
///////////////////////////////////////////
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
//
#include "pcl/impl/instantiate.hpp"
#include "pcl/search/kdtree.h"
#include "pcl/search/impl/kdtree.hpp"
#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"
#include <pcl/registration/lum.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
////////////////////////////////////////////////////////////////////////
#include <pcl/filters/filter.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

namespace Processors {
namespace ClosedCloudMerge {


ClosedCloudMerge::ClosedCloudMerge(const std::string & name) :
    Base::Component(name),
    prop_ICP_alignment("ICP.Points", true),
    prop_ICP_alignment_normal("ICP.Normals",true),
    prop_ICP_alignment_color("ICP.Color",false),
    ICP_transformation_epsilon("ICP.Tranformation_epsilon",1e-6),
    ICP_max_correspondence_distance("ICP.Correspondence_distance",0.1),
    ICP_max_iterations("ICP.Iterations",2000),
    RanSAC_inliers_threshold("RanSac.Inliers_threshold",0.01f),
    RanSAC_max_iterations("RanSac.Iterations",2000),
    viewNumber("View.Number", 5),
    maxIterations("Interations.Max", 5),
    corrTreshold("Correspondenc.Treshold", 10),
    useSHOT("useSHOT", false)
{
    registerProperty(prop_ICP_alignment);
    registerProperty(prop_ICP_alignment_normal);
    registerProperty(prop_ICP_alignment_color);
    registerProperty(ICP_transformation_epsilon);
    registerProperty(ICP_max_correspondence_distance);
    registerProperty(ICP_max_iterations);
    registerProperty(RanSAC_inliers_threshold);
    registerProperty(RanSAC_max_iterations);
    registerProperty(maxIterations);
    registerProperty(viewNumber);
    registerProperty(corrTreshold);
    registerProperty(useSHOT);

	properties.ICP_transformation_epsilon = ICP_transformation_epsilon;
	properties.ICP_max_iterations = ICP_max_iterations;
	properties.ICP_max_correspondence_distance = ICP_max_correspondence_distance;
	properties.RanSAC_inliers_threshold = RanSAC_inliers_threshold;
	properties.RanSAC_max_iterations = RanSAC_max_iterations;
}

ClosedCloudMerge::~ClosedCloudMerge() {
}


void ClosedCloudMerge::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzrgb_normals", &in_cloud_xyzrgb_normals);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
    registerStream("in_cloud_xyzshot", &in_cloud_xyzshot);
	registerStream("out_instance", &out_instance);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb_normals", &out_cloud_xyzrgb_normals);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
    registerStream("out_cloud_xyzshot", &out_cloud_xyzshot);

	registerStream("out_mean_viewpoint_features_number", &out_mean_viewpoint_features_number);

    h_addViewToModel.setup(boost::bind(&ClosedCloudMerge::addViewToModel, this));
    registerHandler("addViewToModel", &h_addViewToModel);
    addDependency("addViewToModel", &in_cloud_xyzsift);

    addDependency("addViewToModel", &in_cloud_xyzrgb_normals);
}

bool ClosedCloudMerge::onInit() {
	// Number of viewpoints.
	counter = 0;
	// Mean number of features per view.
	mean_viewpoint_features_number = 0;

	global_trans = Eigen::Matrix4f::Identity();

	cloud_merged = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_sift_merged = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
    cloud_shot_merged = pcl::PointCloud<PointXYZSHOT>::Ptr(new pcl::PointCloud<PointXYZSHOT>());
	cloud_normal_merged = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	return true;
}

bool ClosedCloudMerge::onFinish() {
	return true;
}

bool ClosedCloudMerge::onStop() {
	return true;
}

bool ClosedCloudMerge::onStart() {
	return true;
}


void ClosedCloudMerge::addViewToModel()
{
    CLOG(LINFO) << "ClosedCloudMerge::addViewToModel";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = in_cloud_xyzrgb_normals.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift = in_cloud_xyzsift.read();
    pcl::PointCloud<PointXYZSHOT>::Ptr cloud_shot;
    if(useSHOT){
        if(in_cloud_xyzshot.empty()){
            CLOG(LERROR) << "ClosedCloudMerge: No XYZSHOT cloud";
            return;
        }
        cloud_shot = in_cloud_xyzshot.read();
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());  //keypoint SIFT i SHOT


	CLOG(LINFO) << "cloud_xyzrgb size: "<<cloudrgb->size();
	CLOG(LINFO) << "cloud_xyzrgb_normals size: "<<cloud->size();
	CLOG(LINFO) << "cloud_xyzsift size: "<<cloud_sift->size();
    if(useSHOT)
        CLOG(LINFO) << "cloud_xyzshot size: "<<cloud_shot->size();

	// Remove NaNs.
	std::vector<int> indices;
	cloudrgb->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloudrgb, *cloudrgb, indices);
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	cloud_sift->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_sift, *cloud_sift, indices);
    if(useSHOT){
        cloud_shot->is_dense = false;
        pcl::removeNaNFromPointCloud(*cloud_shot, *cloud_shot, indices);
    }

	CLOG(LDEBUG) << "cloud_xyzrgb size without NaN: "<<cloud->size();
	CLOG(LDEBUG) << "cloud_xyzsift size without NaN: "<<cloud_sift->size();

	counter++;
	CLOG(LINFO) << "view number: "<<counter;
	CLOG(LINFO) << "view cloud->size(): "<<cloud->size();
	CLOG(LINFO) << "view cloud_sift->size(): "<<cloud_sift->size();

	rgb_views.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>()));
	rgbn_views.push_back(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>()));
    sift_views.push_back(pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>()));
    if(useSHOT)
        shot_views.push_back(pcl::PointCloud<PointXYZSHOT>::Ptr (new pcl::PointCloud<PointXYZSHOT>()));

	// First cloud.
	if (counter == 1)
	{
		//counter++;
	//	mean_viewpoint_features_number = cloud_sift->size();
		// Push results to output data ports.
		out_mean_viewpoint_features_number.write(cloud_sift->size());

		lum_sift.addPointCloud(cloud_sift);
		*rgbn_views[0] = *cloud;
		*rgb_views[0] = *cloudrgb;


		*cloud_merged = *cloudrgb;
		*cloud_normal_merged = *cloud;
		*cloud_sift_merged = *cloud_sift;
        if(useSHOT){
            *cloud_shot_merged = *cloud_shot;
            //tworzymy chmure punktow kluczowych SIFT i SHOT (PointXYZ)
            pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_shot(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::copyPointCloud(*cloud_sift, *keypoints);
            pcl::copyPointCloud(*cloud_shot, *keypoints_shot);
            *keypoints += *keypoints_shot;  //dopisujemy punkty kluczowe SHOT do SIFT
            lum_xyz.addPointCloud(keypoints);
            *sift_views[0] = *cloud_sift;
            *shot_views[0] = *cloud_shot;
        }

		out_cloud_xyzrgb.write(cloud_merged);
		out_cloud_xyzrgb_normals.write(cloud_normal_merged);
		out_cloud_xyzsift.write(cloud_sift_merged);
        if(useSHOT)
            out_cloud_xyzshot.write(cloud_shot_merged);

		// Push SOM - depricated.
//		out_instance.write(produce());
		CLOG(LINFO) << "return ";
		return;
	}
//	 Find corespondences between feature clouds.
//	 Initialize parameters.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
    pcl::CorrespondencesPtr correspondences_shot(new pcl::Correspondences()) ;
	MergeUtils::computeCorrespondences(cloud_sift, cloud_sift_merged, correspondences);
    CLOG(LINFO) << "  correspondences SIFT: " << correspondences->size() ;

    if(useSHOT){
        MergeUtils::computeCorrespondences(cloud_shot, cloud_shot_merged, correspondences_shot);
        CLOG(LINFO) << "  correspondences shot: " << correspondences_shot->size() ;

        //tworzymy chmure punktow kluczowych SIFT i SHOT (PointXYZ)
        pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_shot(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*cloud_sift, *keypoints);
        pcl::copyPointCloud(*cloud_shot, *keypoints_shot);
        *keypoints += *keypoints_shot;  //dopisujemy punkty kluczowe SHOT do SIFT

        //dodajemy dopasowania SHOT do dopasowan SIFT
        for (int i = 0; i < correspondences_shot->size(); ++i) {
            pcl::Correspondence old = correspondences_shot->at(i);
            correspondences->push_back(pcl::Correspondence(old.index_query + cloud_sift->size(), old.index_match + cloud_sift_merged->size(), old.distance));
            //TODO czy distance z SIFT i SHOT rownowazne ?????
        }


}

    CLOG(LINFO) << "  correspondences: " << correspondences->size();
    // Compute transformation between clouds and SOMGenerator global transformation of cloud.
	pcl::Correspondences inliers;
    Eigen::Matrix4f current_trans;
    if(useSHOT){
        //transformacja miedzy keypointami sift i shot
        pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_merged(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_shot_merged(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*cloud_sift_merged, *keypoints_merged);
        pcl::copyPointCloud(*cloud_shot_merged, *keypoints_shot_merged);
        *keypoints_merged += *keypoints_shot_merged;
        current_trans = MergeUtils::computeTransformationSAC(keypoints, keypoints_merged, correspondences, inliers, properties);
    }
    else{
        current_trans = MergeUtils::computeTransformationSAC(cloud_sift, cloud_sift_merged, correspondences, inliers, properties);
    }
	if (current_trans == Eigen::Matrix4f::Identity())
	{
		CLOG(LINFO) << "cloud couldn't be merged";
		counter--;
		out_cloud_xyzrgb.write(cloud_merged);
		out_cloud_xyzrgb_normals.write(cloud_normal_merged);
		out_cloud_xyzsift.write(cloud_sift_merged);

		// Push SOM - depricated.
//		out_instance.write(produce());
		return;
	}

	pcl::transformPointCloud(*cloud, *cloud, current_trans);
	pcl::transformPointCloud(*cloudrgb, *cloudrgb, current_trans);
	pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);
    if(useSHOT){
        pcl::transformPointCloud(*cloud_shot, *cloud_shot, current_trans);
    }


	CLOG(LINFO) << "current trans: \n" << current_trans;

//	current_trans = MergeUtils::computeTransformationICPNormals(cloud, cloud_normal_merged, properties);

//	pcl::transformPointCloud(*cloud, *cloud, current_trans);
//	pcl::transformPointCloud(*cloudrgb, *cloudrgb, current_trans);
//	pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);


	lum_sift.addPointCloud(cloud_sift);
	*rgbn_views[counter -1] = *cloud;
	*rgb_views[counter -1] = *cloudrgb;

    if(useSHOT){
        lum_xyz.addPointCloud(keypoints);
        *sift_views[counter -1] = *cloud_sift;
        *shot_views[counter -1] = *cloud_shot;
    }


	int added = 0;
	for (int i = counter - 2 ; i >= 0; i--)
	{
        if(useSHOT){
            pcl::CorrespondencesPtr correspondences2(new pcl::Correspondences()) ;
            pcl::CorrespondencesPtr correspondences2_shot(new pcl::Correspondences()) ;
            MergeUtils::computeCorrespondences(sift_views[counter - 1], sift_views[i], correspondences2);
            MergeUtils::computeCorrespondences(shot_views[counter - 1], shot_views[i], correspondences2_shot);
            //dodajemy dopasowania SHOT do dopasowan SIFT
            for (int j = 0; j < correspondences2_shot->size(); ++j) {
                pcl::Correspondence old = correspondences2_shot->at(j);
                correspondences2->push_back(pcl::Correspondence(old.index_query + sift_views[counter - 1]->size(), old.index_match + sift_views[i]->size(), old.distance/(float)70000));
                //TODO sprawdzic skad 70000
            }
            pcl::CorrespondencesPtr correspondences3(new pcl::Correspondences()) ;
            MergeUtils::computeTransformationSAC(lum_xyz.getPointCloud(counter - 1), lum_xyz.getPointCloud(i), correspondences2, *correspondences3, properties) ;
            CLOG(LINFO) << "  correspondences3: " << correspondences3->size() << " out of " << correspondences2->size();
            if (correspondences3->size() > corrTreshold) {
                lum_xyz.setCorrespondences(counter-1, i, correspondences3);
                added++;
            }
        }
        else{
            pcl::CorrespondencesPtr correspondences2(new pcl::Correspondences()) ;
            MergeUtils::computeCorrespondences(lum_sift.getPointCloud(counter - 1), lum_sift.getPointCloud(i), correspondences2);
            pcl::CorrespondencesPtr correspondences3(new pcl::Correspondences()) ;
            MergeUtils::computeTransformationSAC(lum_sift.getPointCloud(counter - 1), lum_sift.getPointCloud(i), correspondences2, *correspondences3, properties) ;
            //cortab[counter-1][i] = inliers2;
            CLOG(LINFO) << "  correspondences3: " << correspondences3->size() << " out of " << correspondences2->size();
            if (correspondences3->size() > corrTreshold) {
                lum_sift.setCorrespondences(counter-1, i, correspondences3);
                added++;
            }
        }
	//break;
	//CLOG(LINFO) << "computet for "<<counter-1 <<" and "<< i << "  correspondences2: " << correspondences2->size() << " out of " << correspondences2->size();
	}
	CLOG(LINFO) << "view " << counter << " have correspondences with " << added << " views";
	if (added == 0 )
		CLOG(LINFO) << " Non corespondences found" <<endl;


//	*cloud_merged = *(rgb_views[0]);
//	*cloud_normal_merged = *(rgbn_views[0]);

	if (counter > viewNumber) {
        if(useSHOT){
            lum_xyz.setMaxIterations(maxIterations);
            lum_xyz.compute();
            CLOG(LINFO) << "ended";
            CLOG(LINFO) << "cloud_merged from LUM ";
            for (int i = 1 ; i < viewNumber; i++)
            {
                pcl::PointCloud<pcl::PointXYZRGB> tmprgb = *(rgb_views[i]);
                pcl::PointCloud<pcl::PointXYZRGBNormal> tmp = *(rgbn_views[i]);
                pcl::PointCloud<PointXYZSIFT> tmpsift = *(sift_views[i]);
                pcl::PointCloud<PointXYZSHOT> tmpshot = *(shot_views[i]);
                pcl::transformPointCloud(tmp, tmp, lum_xyz.getTransformation (i));
                pcl::transformPointCloud(tmprgb, tmprgb, lum_xyz.getTransformation (i));
                pcl::transformPointCloud(tmpsift, tmpsift, lum_xyz.getTransformation (i));
                pcl::transformPointCloud(tmpshot, tmpshot, lum_xyz.getTransformation (i));
                *cloud_merged += tmprgb;
                *cloud_normal_merged += tmp;
                *cloud_sift_merged += tmpsift;
                *cloud_shot_merged += tmpshot;
            }
        }
        else{
            lum_sift.setMaxIterations(maxIterations);
            lum_sift.compute();
            cloud_sift_merged = lum_sift.getConcatenatedCloud ();
            CLOG(LINFO) << "ended";
            CLOG(LINFO) << "cloud_merged from LUM ";
            for (int i = 1 ; i < viewNumber; i++)
            {
                pcl::PointCloud<pcl::PointXYZRGB> tmprgb = *(rgb_views[i]);
                pcl::PointCloud<pcl::PointXYZRGBNormal> tmp = *(rgbn_views[i]);
                pcl::transformPointCloud(tmp, tmp, lum_sift.getTransformation (i));
                pcl::transformPointCloud(tmprgb, tmprgb, lum_sift.getTransformation (i));
                *cloud_merged += tmprgb;
                *cloud_normal_merged += tmp;
            }

            // Delete points.
            pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloud_sift_merged->begin();
            while(pt_iter!=cloud_sift_merged->end()){
                if(pt_iter->multiplicity==-1){
                    pt_iter = cloud_sift_merged->erase(pt_iter);
                } else {
                    ++pt_iter;
                }
            }
            if(useSHOT){
                pcl::PointCloud<PointXYZSHOT>::iterator pt_iter = cloud_shot_merged->begin();
                while(pt_iter!=cloud_shot_merged->end()){
                    if(pt_iter->multiplicity==-1){
                        pt_iter = cloud_shot_merged->erase(pt_iter);
                    } else {
                        ++pt_iter;
                    }
                }
            }
        }
	} else {
//		for (int i = 1 ; i < counter; i++)
//        {//po co milion razy kopiowac?? //TODO
//			pcl::PointCloud<pcl::PointXYZRGB> tmprgb = *(rgb_views[i]);
//			pcl::PointCloud<pcl::PointXYZRGBNormal> tmp = *(rgbn_views[i]);
//			pcl::transformPointCloud(tmp, tmp, lum_sift.getTransformation (i));
//			pcl::transformPointCloud(tmprgb, tmprgb, lum_sift.getTransformation (i));
//			*cloud_merged += tmprgb;
//			*cloud_normal_merged += tmp;

//		}
		CLOG(LINFO) << "cloud added ";

        *cloud_merged += *cloudrgb;
        *cloud_normal_merged += *cloud;

        if(useSHOT){
            *cloud_sift_merged += *cloud_sift;
            *cloud_shot_merged += *cloud_shot;           
        }
        else{
            cloud_sift_merged = lum_sift.getConcatenatedCloud ();
        }
	}

	CLOG(LINFO) << "model cloud_merged->size(): "<< cloud_merged->size();
	CLOG(LINFO) << "model cloud_normal_merged->size(): "<< cloud_normal_merged->size();
	CLOG(LINFO) << "model cloud_sift_merged->size(): "<< cloud_sift_merged->size();
    if(useSHOT)
        CLOG(LINFO) << "model cloud_shot_merged->size(): "<< cloud_shot_merged->size();


	// Compute mean number of features.
	//mean_viewpoint_features_number = total_viewpoint_features_number/counter;

	// Push results to output data ports.
	out_mean_viewpoint_features_number.write(total_viewpoint_features_number/counter);
	out_cloud_xyzrgb.write(cloud_merged);
	out_cloud_xyzrgb_normals.write(cloud_normal_merged);
	out_cloud_xyzsift.write(cloud_sift_merged);
    out_cloud_xyzshot.write(cloud_shot_merged);

    CLOG(LINFO) << "ClosedCloudMerge::addViewToModel END.";
}

} // namespace ClosedCloudMerge
} // namespace Processors
