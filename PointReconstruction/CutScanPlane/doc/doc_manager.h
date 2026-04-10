#ifndef _DOC_MANAGER_H_
#define _DOC_MANAGER_H_

#include "doc_bim.h"

/**
* \brief class of document manager
* \author Bichen JING
*/
class Doc_Manager
{
public:
	/**
	* \brief constructor
	*/
	Doc_Manager()
		: m_doc_bim(nullptr)
	{
	}
	/**
	* \brief destructor
	*/
	virtual ~Doc_Manager() {
		delete m_doc_bim;
	}
	/**
	* \brief clear
	*/
	void clear() {
		if (m_doc_bim) {
			m_doc_bim->clear();
			Doc_BIM* temp = m_doc_bim;
			m_doc_bim = nullptr;
			delete temp;
		}
	}
	/**
	* \brief set doc_bim
	*/
	void setDocBIM(Doc_BIM *doc_bim) {
		// check and replace model_pc in model manager
		if (m_doc_bim) {
			m_doc_bim->clear();
			Doc_BIM* temp = m_doc_bim;
			m_doc_bim = doc_bim;
			delete temp;
		}
		else {
			m_doc_bim = doc_bim;
		}
	}
	/**
	* \brief get doc_bim
	*/
	Doc_BIM *getDocBIM() {
		return m_doc_bim;
	}
protected:
	Doc_BIM *m_doc_bim;	/**< document of BIM */
};


#endif // !_DOC_MANAGER_H_
