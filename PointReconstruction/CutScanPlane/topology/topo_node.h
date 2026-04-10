#ifndef _TOPO_NODE_H_
#define _TOPO_NODE_H_
#include "topo_edge.h"

class Topo_Edge;

/**
* \brief class of topological attributes of node
* \author  JING Bichen
*/
class Topo_Attr_Node {
public:
	/**
	* \brief constructor
	*/
	Topo_Attr_Node()
		: m_isVisited(false)
	{}
	/**
	* \brief destructor
	*/
	virtual ~Topo_Attr_Node() {}
public:
	bool m_isVisited;	/**< is visited */
};

/**
* \brief class of topological node data
*/
class Topo_Data_Node {
public:
	/**
	* \brief constructor
	*/
	Topo_Data_Node() 
		: m_name("null")
	{}
	/**
	* \brief destructor
	*/
	virtual ~Topo_Data_Node() {}
	/**
	* \brief set name
	*/
	void setName(const std::string name) {
		m_name = name;
	}
	/**
	* \brief get name
	*/
	std::string getName() const {
		return m_name;
	}
private:
	std::string m_name;		/**< name of node */
};

/**
* \brief class of topological node
*/
class Topo_Node {
public:
	typedef Topo_Data_Node DT; // data
	/**
	* \brief constructor
	*/
	Topo_Node()
		: m_first_edge(nullptr)
		, m_id(-1)
	{
		m_attr = new Topo_Attr_Node();
		m_data = new DT();
	}
	/**
	* \brief destructor
	*/
	virtual ~Topo_Node()
	{
		clear();
	}
	/**
	* \brief clear
	*/
	void clear()
	{
		delete m_attr;
	}
	/**
	* \brief set first edge
	*/
	void setFirstEdge(Topo_Edge *first_edge) {
		m_first_edge = first_edge;
	}
	/**
	* \brief get first edge
	*/
	Topo_Edge *getFirstEdge() {
		return m_first_edge;
	}
	/**
	* \brief get attributes
	*/
	Topo_Attr_Node *getAttr() {
		return m_attr;
	}
	/**
	* \brief set id of node
	* default value is -1, means not added into graph, cannot smaller than -1
	*/
	bool setID(const int val) {
		if (val < -1) return false;
		m_id = val;
		return true;
	}
	/**
	* \brief get id of node
	*/
	int getID() const {
		return m_id;
	}
	/**
	* \brief get data
	*/
	DT *getData() {
		return m_data;
	}
public:
	// connection
	Topo_Edge *m_first_edge;/**< first edge linked with node */
	// information
	int m_id;				/**< index of node as a unique label, used for quick mapping to this node */
	Topo_Attr_Node *m_attr;	/**< topological attributes of node */
	DT *m_data;				/**< data of node */
};

#endif