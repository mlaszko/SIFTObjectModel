/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "HomogMatrixVector.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace HomogMatrixVector {

HomogMatrixVector::HomogMatrixVector(const std::string & name) :
		Base::Component(name)  {

}

HomogMatrixVector::~HomogMatrixVector() {
}

void HomogMatrixVector::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_hm", &in_hm);
	registerStream("in_hms", &in_hms);
	registerStream("out_hms", &out_hms);
	// Register handlers
	registerHandler("create_vector", boost::bind(&HomogMatrixVector::create_vector, this));
	addDependency("create_vector", &in_hm);
	registerHandler("expand_vector", boost::bind(&HomogMatrixVector::expand_vector, this));
	addDependency("expand_vector", &in_hms);
	addDependency("expand_vector", &in_hm);

}

bool HomogMatrixVector::onInit() {

	return true;
}

bool HomogMatrixVector::onFinish() {
	return true;
}

bool HomogMatrixVector::onStop() {
	return true;
}

bool HomogMatrixVector::onStart() {
	return true;
}

void HomogMatrixVector::create_vector() {
    CLOG(LTRACE) << "create_vector";
    vector<Types::HomogMatrix> hms;
    while(!in_hm.empty()){
        Types::HomogMatrix hm = in_hm.read();
        hms.push_back(hm);
    }
    out_hms.write(hms);
}

void HomogMatrixVector::expand_vector() {
    CLOG(LTRACE) << "expand_vector";
    vector<Types::HomogMatrix> hms = in_hms.read();
    while(!in_hm.empty()){
        Types::HomogMatrix hm = in_hm.read();
        hms.push_back(hm);
    }
    out_hms.write(hms);
}



} //: namespace HomogMatrixVector
} //: namespace Processors
