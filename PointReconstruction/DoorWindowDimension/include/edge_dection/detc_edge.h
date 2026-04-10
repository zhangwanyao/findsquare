#ifndef _DETC_EDGE_H_
#define _DETC_EDGE_H_
//#include "detc_feature.h"

/**
* \brief base class for detect edge operation
*/

class Detc_Edge {
public:
	/**
	* \brief constructor
	*/
	Detc_Edge() {}
	/**
	* \brief destructor
	*/
	~Detc_Edge() {}
	/**
	* \brief initialize class member
	* @return if class member are initialized or not
	*/
	virtual bool init() = 0;
	/**
	* \brief detect feature
	*/
	virtual bool detect() = 0;
	/**
	* \brief clear buffer
	*/
	virtual void clear() = 0;
};

#endif // !_DECT_EDGE_H_
