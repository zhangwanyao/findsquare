#ifndef _DOC_H_
#define _DOC_H_

/**
* \brief base class of document
* \author Bichen JING
*/
class Doc {
public:
	/**
	* \brief constructor
	*/
	Doc() {}
	/**
	* \brief destructor
	*/
	virtual ~Doc() {}
	/**
	* \brief pure virtual function, clear data
	*/
	virtual void clear() = 0;
	/**
	* \brief pure virtual function, update model
	*/
	virtual void update() = 0;
};

#endif