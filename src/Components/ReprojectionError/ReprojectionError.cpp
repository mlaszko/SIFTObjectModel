/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "ReprojectionError.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ReprojectionError {

ReprojectionError::ReprojectionError(const std::string & name) :
		Base::Component(name)  {

}

ReprojectionError::~ReprojectionError() {
}

void ReprojectionError::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_rototranslations", &in_rototranslations);
//	registerStream("in_location", &in_location);
    registerStream("in_location_hm", &in_location_hm);
	// Register handlers
//    registerHandler("calculate_errors_eigen", boost::bind(&ReprojectionError::calculate_errors_eigen, this));
//    addDependency("calculate_errors_eigen", &in_rototranslations);
//    addDependency("calculate_errors_eigen", &in_location);
    registerHandler("calculate_errors_hm", boost::bind(&ReprojectionError::calculate_errors_hm, this));
    addDependency("calculate_errors_hm", &in_rototranslations);
    addDependency("calculate_errors_hm", &in_location_hm);

}

bool ReprojectionError::onInit() {

	return true;
}

bool ReprojectionError::onFinish() {
	return true;
}

bool ReprojectionError::onStop() {
	return true;
}

bool ReprojectionError::onStart() {
	return true;
}

//void ReprojectionError::calculate_errors_eigen() {
//    CLOG(LTRACE) << "ReprojectionError::calculate_errors_eigen";
//    std::vector<Types::HomogMatrix> rototranslations = in_rototranslations.read();
//    Eigen::Matrix4d location = in_location.read();
//    calculate_errors(rototranslations, location);
//}

void ReprojectionError::calculate_errors_hm() {
    CLOG(LTRACE) << "ReprojectionError::calculate_errors_hm";
    std::vector<Types::HomogMatrix> rototranslations = in_rototranslations.read();
    Types::HomogMatrix location = in_location_hm.read();
//    Eigen::Matrix4d l =  Eigen::Matrix4d::Identity();
//    for(int i = 0; i < 4; i++)
//        for(int j = 0; j < 4; j++){
//            l(i,j) = location(i, j);
//        }
    calculate_errors(rototranslations, location);
}
//min{|a−b|, 2π−|a−b|}
double d(double a, double b){
    double d1 = abs(a-b);
    double d2 = 2*M_PI - d1;
    return d1 < d2 ? d1 : d2;
}

void ReprojectionError::calculate_errors(std::vector<Types::HomogMatrix> rototranslations, Types::HomogMatrix location) {
    CLOG(LTRACE) << "ReprojectionError::calculate_errors";
    CLOG(LDEBUG) << "Ground truth:\n" << location ;
    Types::HomogMatrix inv;
    inv.matrix() = location.matrix().inverse();
    CLOG(LDEBUG) << "Ground truth inversed:\n" << inv ;

    if(rototranslations.empty()){
        CLOG(LINFO)<< "ReprojectionError No hypotheses available" ;
    }
    for(int i = 0; i < rototranslations.size(); i++){
        CLOG(LDEBUG)<< "Object Translation " << i << endl << rototranslations[i]<<endl;
        Types::HomogMatrix dT;
        dT.matrix() = rototranslations[i].matrix() * inv.matrix();
        CLOG(LDEBUG)<< "dT" << endl << dT <<endl;

        double et = sqrt( pow(dT(0,3), 2) + pow(dT(1,3), 2) + pow(dT(2,3), 2));
        double er = acos((dT.matrix().trace()-1-1)/2);
        CLOG(LINFO)<< "ReprojectionErrorOLD Hypothese " << i << " et "<< et << " er " << er<< " x " << dT(0,3) << " y " << dT(1,3) << " z " << dT(2,3) << endl;

        cv::Mat_<double> rotationMatrix_in1 = cv::Mat_<double>::zeros(3,3);
        cv::Mat_<double> rotationMatrix_in2 = cv::Mat_<double>::zeros(3,3);
        cv::Mat_<double> rotation1;
        cv::Mat_<double> rotation2;
        // Copy input values to temporary matrix.
        for (int k = 0; k < 3; ++k) {
            for (int j = 0; j < 3; ++j) {
                rotationMatrix_in1(k,j) = rototranslations[i](k,j);
                rotationMatrix_in2(k,j) = location(k,j);
            }
        }
        // Compute rotation vector.
        Rodrigues(rotationMatrix_in1, rotation1);
        Rodrigues(rotationMatrix_in2, rotation2);
        double er2 = sqrt( pow(d(rotation1(0,0), rotation2(0,0)), 2) + pow(d(rotation1(0,1), rotation2(0,1)), 2) + pow(d(rotation1(0,2), rotation2(0,2)), 2)) ;

        //Translation error
        double x = rototranslations[i](0,3) - location(0,3);
        double y = rototranslations[i](1,3) - location(1,3);
        double z = rototranslations[i](2,3) - location(2,3);
        double et2 = sqrt( pow(x, 2) + pow(y, 2) + pow(z, 2));
        CLOG(LINFO)<< "ReprojectionError Hypothese " << i << " et " << et2 << " er " << er2 << " x " << x << " y " << y << " z " << z << endl;
    }
}



} //: namespace ReprojectionError
} //: namespace Processors
