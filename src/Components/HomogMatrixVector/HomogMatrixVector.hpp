/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef HOMOGMATRIXVECTOR_HPP_
#define HOMOGMATRIXVECTOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "Types/HomogMatrix.hpp"

namespace Processors {
namespace HomogMatrixVector {

/*!
 * \class HomogMatrixVector
 * \brief HomogMatrixVector processor class.
 *
 * 
 */
class HomogMatrixVector: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	HomogMatrixVector(const std::string & name = "HomogMatrixVector");

	/*!
	 * Destructor
	 */
	virtual ~HomogMatrixVector();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

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
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams
	Base::DataStreamIn<Types::HomogMatrix> in_hm;
	Base::DataStreamIn<vector<Types::HomogMatrix> > in_hms;

	// Output data streams
	Base::DataStreamOut<vector<Types::HomogMatrix> > out_hms;

	// Handlers

	// Properties

	
	// Handlers
	void create_vector();
	void expand_vector();

};

} //: namespace HomogMatrixVector
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("HomogMatrixVector", Processors::HomogMatrixVector::HomogMatrixVector)

#endif /* HOMOGMATRIXVECTOR_HPP_ */
