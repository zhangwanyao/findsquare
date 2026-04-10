#ifndef _TOPO_GRAPH_H_
#define _TOPO_GRAPH_H_
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include "topo_node.h"
#include "topo_edge.h"

/**
* \brief class of topological attributes of graph
*/
class Topo_Attr_Graph {
public:
	/**
	* \brief constructor
	*/
	Topo_Attr_Graph() {}
	/**
	* \brief destructor
	*/
	virtual ~Topo_Attr_Graph() {}
public:

};

/**
* \brief class of topological graph
*/
class Topo_Graph {
public:
	typedef int* DT;
	/**
	* \brief constructor
	*/
	Topo_Graph() 
		: m_name("default")
		, m_data(nullptr)
		, m_attr(nullptr)
	{
		m_attr = new Topo_Attr_Graph();
	}
	/**
	* \brief destructor
	*/
	virtual ~Topo_Graph() 
	{
		clear();
		delete m_attr;
	}
	/**
	* \brief clear
	*/
	void clear() 
	{
		for (unsigned int i = 0; i < m_edges.size(); i++) {
			if (m_edges[i] != nullptr) {
				m_edges[i]->clear();
			}
		}
		for (unsigned int i = 0; i < m_nodes.size(); i++) {
			if (m_nodes[i] != nullptr) {
				m_nodes[i]->clear();
			}
		}
		m_edges.clear();
		m_nodes.clear();
		clearAttr();
	}
	/**
	* \brief clear graph attributes
	*/
	void clearAttr() {
		if (m_attr != nullptr) {
			auto temp = m_attr;
			m_attr = nullptr;
			delete temp;
			m_attr = new Topo_Attr_Graph();
		}
	}
	/**
	* \brief update node and edge info
	*/
	void update() {
		update_ID();
	}
	/**
	* \brief update id of node and edge
	*/
	void update_ID() {
		for (unsigned int i = 0; i < m_nodes.size(); i++) {
			m_nodes[i]->setID(i);
		}
		for (unsigned int i = 0; i < m_edges.size(); i++) {
			m_edges[i]->setID(i);
		}
	}
	/**
	* \brief create graph from node list and edges
	* \param[in] nodes node list
	* \param[in] edges node pairs, idx pairs in node list
	*/
	bool create(const std::vector<Topo_Node *> &nodes
		, const std::vector<std::pair<unsigned int, unsigned int>> &edges)
	{
		clear();
		// data validation
		size_t nnode = nodes.size();
		size_t nedge = edges.size();
		if (nnode < 1) return false;
		// number of edge > edge of complete graph
		if (nedge > (nnode * (nnode - 1) * 0.5)) return false;
		
		// temporary vector of last edge related to each node
		// update when inserting a new edge
		std::vector<Topo_Edge *> last_edges;
		last_edges.resize(nnode);
		std::fill(last_edges.begin(), last_edges.end(), nullptr);
		
		// initiate edge and link
		for (size_t i = 0; i < nedge; i++)
		{
			unsigned int p = edges[i].first;
			unsigned int q = edges[i].second;
			Topo_Edge *temp_edge = new Topo_Edge();
			temp_edge->m_node_p = nodes[p];
			temp_edge->m_node_q = nodes[q];
			// if node has no first edge, link with node
			if (nodes[p]->getFirstEdge() == nullptr) nodes[p]->setFirstEdge(temp_edge);
			if (nodes[q]->getFirstEdge() == nullptr) nodes[q]->setFirstEdge(temp_edge);
			// update last edge
			if (last_edges[p] == nullptr) {
				last_edges[p] = temp_edge;
			}else {
				// compare node[p] with member node p and q, find same node then update related next edge
				if (nodes[p] == last_edges[p]->m_node_p) {
					last_edges[p]->m_next_edge_p = temp_edge;
				}
				else {
					last_edges[p]->m_next_edge_q = temp_edge;
				}
				// update last edge
				last_edges[p] = temp_edge;
			}
			// same operation on node[q]
			if (last_edges[q] == nullptr) {
				last_edges[q] = temp_edge;
			}else{
				if (nodes[q] == last_edges[q]->m_node_p) {
					last_edges[q]->m_next_edge_p = temp_edge;
				}
				else {
					last_edges[q]->m_next_edge_q = temp_edge;
				}
				last_edges[q] = temp_edge;
			}
			m_edges.push_back(temp_edge);
		}
		// copy data to instance member
		m_nodes = nodes;
		last_edges.clear();
		update();
		return true;
	}
protected:
	/**
	* \brief insert edge containing node
	*/
	bool insertEdge(Topo_Node *node, Topo_Edge *edge) {
		// check if node already in graph
		bool isNodeInGraph = false;
		for (unsigned int i = 0; i < m_nodes.size(); i++) {
			if (m_nodes[i] == node) {
				isNodeInGraph = true;
				break;
			}
		}
		// if not, new edge is first edge of node, append to node list
		if (!isNodeInGraph) {
			node->m_first_edge = edge;
			m_nodes.push_back(node);
			return true;
		}
		// if yes, find last edge containing node, then link the new edge
		else {
			Topo_Edge *prev_edge = nullptr;
			Topo_Edge *cur_edge = node->getFirstEdge();
			while (cur_edge != nullptr) {
				prev_edge = cur_edge; // keep edge not null for linking
				// compare nodes in cur_edge with target node
				if (cur_edge->m_node_p == node) {
					cur_edge = cur_edge->m_next_edge_p;
					continue;
				}
				if (cur_edge->m_node_q == node) {
					cur_edge = cur_edge->m_next_edge_q;
				}
			}
			if (prev_edge != nullptr) {// link edge
				cur_edge = prev_edge;
				if (cur_edge->m_node_p == node) {
					cur_edge->m_next_edge_p = edge;
					return true;
				}
				if (cur_edge->m_node_q == node) {
					cur_edge->m_next_edge_q = edge;
					return true;
				}
			}
		}
		return false;
	}
public:
	/**
	* \brief insert edge containing two nodes
	*/
	bool insertEdge(Topo_Node *node_p, Topo_Node *node_q)
	{
		// data validation
		// not allow edge to itself, it will have infinite loop
		if (node_p == nullptr || node_q == nullptr || node_p == node_q) {
			return false;
		}
		// init edge
		Topo_Edge *edge = new Topo_Edge();
		edge->m_node_p = node_p;
		edge->m_node_q = node_q;
		// insert edge for both node_p and q
		if (insertEdge(node_p, edge) && insertEdge(node_q, edge)) {
			m_edges.push_back(edge);
			update();
			return true;
		}
		return false;
	}

	/**
	* \brief insert node
	* \param[in] nbr_nodes connected nodes of node
	*/
	bool insertNode(Topo_Node *node, std::vector<Topo_Node *> nbr_nodes) {
		// data validation
		if (node == nullptr) return false;
		unsigned int nnbr = nbr_nodes.size();
		if (nnbr < 1) return false;
		// check if nbr_nodes in graph, if not return false
		unsigned int num_matched = 0;
		for (unsigned int i = 0; i < nnbr; i++) {
			for (unsigned int idx = 0; idx < m_nodes.size(); idx++) {
				if (nbr_nodes[i] == m_nodes[idx]) {
					num_matched++;
					break;
				}
			}
		}
		if (num_matched != nnbr) return false; // nbr_nodes not all in graph, error
		for (unsigned int i = 0; i < nnbr; i++) {
			insertEdge(node, nbr_nodes[i]);
		}
		update();
		return true;
	}
	/**
	* \brief remove node
	* remove node need to remove the connection of neighboring nodes with target node
	*/
	bool removeNode(Topo_Node *node) {
		// data validation
		if (node == nullptr) return false;
		// check if node in graph
		bool isNodeInGraph = false;
		int id_node = -1;
		for (int i = 0; i < m_nodes.size(); i++) {
			if (m_nodes[i] == node) {
				isNodeInGraph = true;
				id_node = i;
				break;
			}
		}
		if (!isNodeInGraph) return false; // node not in graph
		// get neighoring nodes
		std::vector<Topo_Node *> nbr_nodes;
		getNbrNodes(node, nbr_nodes);
		// for each nbr_node, pop edge containing node, namely nbr_edge of node, 
		// , then remove popped edge in m_edges
		// Note: link next edge
		std::vector<Topo_Edge *> nbr_edges;
		for (unsigned int i = 0; i < nbr_nodes.size(); i++) {
			Topo_Node *nbr_node = nbr_nodes[i];
			Topo_Edge *prev_edge = nullptr;
			Topo_Edge *cur_edge = nbr_node->getFirstEdge();
			while (cur_edge != nullptr) {
				// if current edge containing target node
				if (cur_edge->m_node_p == node) { // m_node_q = nbr_node
					nbr_edges.push_back(cur_edge);
					// if prev_edge is null, reset first edge of nbr_node
					if (prev_edge == nullptr) {
						nbr_node->setFirstEdge(cur_edge->m_next_edge_q);
					}
					else {
						if (prev_edge->m_node_p == nbr_node) {
							prev_edge->m_next_edge_p = cur_edge->m_next_edge_q;
						}
						else {
							prev_edge->m_next_edge_q = cur_edge->m_next_edge_q;
						}
					}
					break;
				}
				else if (cur_edge->m_node_q == node) { // m_node_p = nbr_node
					nbr_edges.push_back(cur_edge);
					// if prev_edge is null, reset first edge of nbr_node
					if (prev_edge == nullptr) {
						nbr_node->setFirstEdge(cur_edge->m_next_edge_p);
					}
					else {
						if (prev_edge->m_node_p == nbr_node) {
							prev_edge->m_next_edge_p = cur_edge->m_next_edge_p;
						}
						else {
							prev_edge->m_next_edge_q = cur_edge->m_next_edge_p;
						}
					}
					break;
				}
				// if current edge not containing target node, move to next edge containing nbr_node
				else {
					prev_edge = cur_edge;
					if (cur_edge->m_node_p == nbr_node) {
						cur_edge = cur_edge->m_next_edge_p;
					}
					else {
						cur_edge = cur_edge->m_next_edge_q;
					}
					continue;
				}
			}
		}
		// remove edges
		for (unsigned int i = 0; i < nbr_edges.size(); i++) {
			for (unsigned int j = 0; j < m_edges.size(); j++) {
				if (m_edges[j] == nbr_edges[i]) {
					m_edges.erase(m_edges.begin() + j);
					Topo_Edge *temp = nbr_edges[i];
					nbr_edges[i] = nullptr;
					delete temp;
					break;
				}
			}
		}
		// remove node
		if (id_node > -1) {
			m_nodes.erase(m_nodes.begin() + id_node);
			delete node;
		}
		nbr_nodes.clear();
		nbr_edges.clear();
		update();
		return true;
	}
	/**
	* \brief DFS from given node
	*/
	void DFS(Topo_Node *node, std::vector<Topo_Node *> &visited) 
	{
		node->m_attr->m_isVisited = true;
		visited.push_back(node);
		// get nbr nodes
		std::vector<Topo_Node *> nbr_nodes;
		getNbrNodes(node, nbr_nodes);
		for (size_t i = 0; i < nbr_nodes.size(); ++i) {
			if (!nbr_nodes[i]->m_attr->m_isVisited) {
				DFS(nbr_nodes[i], visited);
			}
		}
		nbr_nodes.clear();
	}
	/**
	* \brief get connected components in graph
	* \param[in] thres_nnode, threshold of number of nodes for a component
	* \param[out] cmpts output components
	*/
	void getCmpts(const unsigned int thres_nnode, std::vector<std::vector<Topo_Node *>> &cmpts) 
	{
		// data validation
		if (m_nodes.empty()) return;
		for (size_t i = 0; i < m_nodes.size(); ++i)
		{
			if (m_nodes[i]->m_attr->m_isVisited) continue;
			std::vector<Topo_Node *> temp_cmpt;
			DFS(m_nodes[i], temp_cmpt);
			if (temp_cmpt.size() > thres_nnode) {
				cmpts.push_back(temp_cmpt);
			}
			temp_cmpt.clear();
		}
	}
	/**
	* \brief get neighboring nodes of target node
	*/
	void getNbrNodes(Topo_Node *node, std::vector<Topo_Node *> &nbr_nodes) {
		nbr_nodes.clear();
		Topo_Edge *cur_edge = node->getFirstEdge();
		if (cur_edge == nullptr) return;
		while (cur_edge != nullptr) {
			if (node == cur_edge->m_node_p) {
				nbr_nodes.push_back(cur_edge->m_node_q);
				cur_edge = cur_edge->m_next_edge_p;
			}
			else {
				nbr_nodes.push_back(cur_edge->m_node_p);
				cur_edge = cur_edge->m_next_edge_q;
			}
		}
	}
	/**
	* \brief get neighboring edges of target node
	*/
	void getNbrEdges(Topo_Node *node, std::vector<Topo_Edge *> &nbr_edges)
	{
		nbr_edges.clear();
		Topo_Edge *cur_edge = node->getFirstEdge();
		if (cur_edge == nullptr) return;
		while (cur_edge != nullptr) {
			nbr_edges.push_back(cur_edge); // append current edge
			if (node == cur_edge->m_node_p) {
				cur_edge = cur_edge->m_next_edge_p;
			}
			else {
				cur_edge = cur_edge->m_next_edge_q;
			}
		}
	}

	/**
	* \brief get nodes
	*/
	std::vector<Topo_Node *> getNodes() {
		return m_nodes;
	}
	/**
	* \brief get edges
	*/
	std::vector<Topo_Edge *> getEdges() {
		return m_edges;
	}
	/**
	* \brief get number of nodes
	*/
	unsigned int getNumNodes() {
		return m_nodes.size();
	}
	/**
	* \brief get number of edges
	*/
	unsigned int getNumEdges() {
		return m_edges.size();
	}

private:
	// topology
	std::vector<Topo_Node *> m_nodes;	/**< all nodes */
	std::vector<Topo_Edge *> m_edges;	/**< all edges */
	// information
	std::string m_name;					/**< name of graph */
	Topo_Attr_Graph *m_attr;			/**< topological attributes of graph */
	DT m_data;							/**< data of graph */
};


#endif // !_TOPO_GRAPH_H_

//// example: test topological graph ..
//Topo_Graph *graph = new Topo_Graph();
//std::vector<Topo_Node *> nodes;
//nodes.resize(6);
//for (unsigned int i = 0; i < 6; i++) {
//	nodes[i] = new Topo_Node();
//}
//nodes[0]->getData()->setName("floor");
//nodes[1]->getData()->setName("ceiling");
//nodes[2]->getData()->setName("wall_e"); // east
//nodes[3]->getData()->setName("wall_w"); // west
//nodes[4]->getData()->setName("wall_s"); // south
//nodes[5]->getData()->setName("wall_n"); // north

//std::vector<std::pair<unsigned int, unsigned int>> edges;
//edges.push_back(std::make_pair(0, 2));
//edges.push_back(std::make_pair(0, 3));
//edges.push_back(std::make_pair(0, 4));
//edges.push_back(std::make_pair(0, 5));
//edges.push_back(std::make_pair(1, 2));
//edges.push_back(std::make_pair(1, 3));
//edges.push_back(std::make_pair(1, 4));
//edges.push_back(std::make_pair(1, 5));
//edges.push_back(std::make_pair(2, 4));
//edges.push_back(std::make_pair(2, 5));
//edges.push_back(std::make_pair(3, 4));
//edges.push_back(std::make_pair(3, 5));

//graph->create(nodes, edges);
//// test graph topology
//std::vector<Topo_Node *> nbr_nodes;
//graph->getNbrNodes(nodes[2], nbr_nodes);
//for (unsigned int i = 0; i < nbr_nodes.size(); i++) {
//	DBG_OUT << QString::fromStdString(nbr_nodes[i]->getData()->getName());
//}

//Topo_Node *mid_node = new Topo_Node();
//mid_node->getData()->setName("mid");
//nbr_nodes.clear();
//nbr_nodes.push_back(nodes[0]);
//nbr_nodes.push_back(nodes[1]);
//nbr_nodes.push_back(nodes[4]);
//nbr_nodes.push_back(nodes[5]);

//graph->insertNode(mid_node, nbr_nodes);
////graph->insertEdge(mid_node, nodes[4]);
////graph->insertEdge(mid_node, nodes[5]);

//graph->removeNode(mid_node);

//widget_graph->resetGraph(graph);

//delete graph;
//// test topological graph end