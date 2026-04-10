#ifndef _TOPO_EDGE_H_
#define _TOPO_EDGE_H_
#include "topo_node.h"

class Topo_Node;

/**
* \brief class of topological attributes of edge
*/
class Topo_Attr_Edge {
public:
	/**
	* \brief constructor
	*/
	Topo_Attr_Edge()
		: m_isVisited(false)
	{}
	/**
	* \brief destructor
	*/
	virtual ~Topo_Attr_Edge() {}
public:
	bool m_isVisited;	/**< is visited */
};

/**
* \brief class of topological node data
*/
class Topo_Data_Edge {
public:
	/**
	* \brief constructor
	*/
	Topo_Data_Edge()
		: m_name("null")
	{}
	/**
	* \brief destructor
	*/
	virtual ~Topo_Data_Edge() {}
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
	std::string m_name;		/**< name of edge */
};

/**
* \brief class of topological edge
*/
class Topo_Edge {
public:
	typedef Topo_Data_Edge DT; // data
	/**
	* \brief constructor
	*/
	Topo_Edge()
		: m_node_p(nullptr)
		, m_node_q(nullptr)
		, m_next_edge_p(nullptr)
		, m_next_edge_q(nullptr)
		, m_id(-1)
	{
		m_attr = new Topo_Attr_Edge();
		m_data = new DT();
	}
	/**
	* \brief destructor
	*/
	virtual ~Topo_Edge()
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
	* \brief set id of edge
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
	* \brief get attributes
	*/
	Topo_Attr_Edge *getAttr() {
		return m_attr;
	}
	/**
	* \brief get data
	*/
	DT *getData() {
		return m_data;
	}
public:
	// connection
	Topo_Node *m_node_p;		/**< node p */
	Topo_Node *m_node_q;		/**< node q */
	Topo_Edge *m_next_edge_p;	/**< next edge containing node p */
	Topo_Edge *m_next_edge_q;	/**< next edge containing node q */
	// information
	int m_id;					/**< index of edge as a unique label, used for quick mapping to this edge */
	Topo_Attr_Edge *m_attr;		/**< topological attributes of edge */
	DT *m_data;					/**< data of edge */
};

#endif // !_TOPO_EDGE_H_
