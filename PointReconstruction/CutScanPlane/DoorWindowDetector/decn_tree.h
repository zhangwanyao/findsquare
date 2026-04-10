#ifndef _DECN_TREE_H_
#define _DECN_TREE_H_
#include "decn_doc.h"
#include "decn_cond.h"
//#include "../config_dbg.h"

class Decn_Node;
class Decn_Tree;

/**
* \brief class of node of binary decision tree
* \author JING Bichen
*/
class Decn_Node {
public:
	/**
	* \brief type of condition
	*/
	enum class TYPE_COND {
		DEFAULT,
		UNARY,
		BINARY,
		TERNARY,
		NARRAY
	};
	/**
	* \brief constructor
	*/
	Decn_Node() 
		: m_left(nullptr)
		, m_right(nullptr)
		, m_id(0)
		, m_cond(nullptr)
		, m_type_cond(TYPE_COND::DEFAULT)
		, m_idx_data(-1)
		, m_idx_result(-1)
	{
	}
	/**
	* \brief constructor
	*/
	Decn_Node(const unsigned int id)
		: m_left(nullptr)
		, m_right(nullptr)
		, m_id(id)
		, m_cond(nullptr)
		, m_type_cond(TYPE_COND::DEFAULT)
		, m_idx_data(-1)
		, m_idx_result(-1)
	{
	}
	/**
	* \brief destructor
	*/
	virtual ~Decn_Node(){
		clear();
	}
	/**
	* \brief init with unary condition
	*/
	bool init(const int idx_data, const Decn_Cond::TYPE_UNARY &type_unary) {
		clearCond();
		// data validation
		if (idx_data < 0) return false;
		if (type_unary == Decn_Cond::TYPE_UNARY::DEFAULT) return false;
		// set idx_data
		m_idx_data = idx_data;
		// init condition
		m_cond = new Decn_Cond(type_unary);
		m_type_cond = TYPE_COND::UNARY;
		return true;
	}
	/**
	* \brief init with binary condition
	*/
	bool init(const int idx_data, const Decn_Cond::TYPE_BINARY &type_binary) {
		clearCond();
		// data validation
		if (idx_data < 0) return false;
		if (type_binary == Decn_Cond::TYPE_BINARY::DEFAULT) return false;
		// set idx_data
		m_idx_data = idx_data;
		// init condition
		m_cond = new Decn_Cond(type_binary);
		m_type_cond = TYPE_COND::BINARY;
		return true;
	}
	/**
	* \brief init with ternary condition
	*/
	bool init(const int idx_data, const Decn_Cond::TYPE_TERNARY &type_ternary) {
		clearCond();
		// data validation
		if (idx_data < 0) return false;
		if (type_ternary == Decn_Cond::TYPE_TERNARY::DEFAULT) return false;
		// set idx_data
		m_idx_data = idx_data;
		// init condition
		m_cond = new Decn_Cond(type_ternary);
		m_type_cond = TYPE_COND::TERNARY;
		return true;
	}
	/**
	* \brief init with nArray condition
	*/
	bool init(const int idx_data, const Decn_Cond::TYPE_NARRAY &type_narray) {
		clearCond();
		// data validation
		if (idx_data < 0) return false;
		if (type_narray == Decn_Cond::TYPE_NARRAY::DEFAULT) return false;
		// set idx_data
		m_idx_data = idx_data;
		// init condition
		m_cond = new Decn_Cond(type_narray);
		m_type_cond = TYPE_COND::NARRAY;
		return true;
	}
	/**
	* \brief clear
	*/
	void clear() {
		clearCond();
	}
	/**
	* \brief clear condition
	*/
	void clearCond() {
		m_type_cond = TYPE_COND::DEFAULT;
		if (m_cond != nullptr) {
			Decn_Cond *temp = m_cond;
			m_cond = nullptr;
			delete temp;
		}
	}
	/**
	* \brief update node
	*/
	void update() {
		// if node is not leaf node, reset index of result
		if (!isLeaf()) {
			m_idx_result = -1;
		}
	}
	/**
	* \brief conditional checking with input data doc
	*/
	template<class T>
	bool check(const Decn_Doc<T> &data) {
		// data validation
		// condition
		assert(m_cond != nullptr);
		assert(m_type_cond != TYPE_COND::DEFAULT);
		// input data
		assert(m_idx_data > -1);
		switch (m_type_cond) {
		case TYPE_COND::UNARY:
			assert(m_idx_data < data.m_uData.size());
			return m_cond->check(data.m_uData[m_idx_data]);
			break;
		case TYPE_COND::BINARY:
			assert(m_idx_data < data.m_bData.size());
			return m_cond->check(data.m_bData[m_idx_data]);
			break;
		case TYPE_COND::TERNARY:
			assert(m_idx_data < data.m_tData.size());
			return m_cond->check(data.m_tData[m_idx_data]);
			break;
		case TYPE_COND::NARRAY:
			assert(m_idx_data < data.m_naData.size());
			return m_cond->check(data.m_naData[m_idx_data]);
			break;
		default:
			break;
		}
		return false;
	}
	/**
	* \brief append constraint
	*/
	template<class UT>
	void appendCSTR(const UT &val) {
		m_cond->appendCSTR(val);
	}
	/**
	* \brief append constraint
	*/
	template<class BT>
	void appendCSTR(const std::pair<BT, BT> &val) {
		m_cond->appendCSTR(val);
	}
	/**
	* \brief append constraint
	*/
	template<class TT>
	void appendCSTR(const std::tuple<TT, TT, TT> &val) {
		m_cond->appendCSTR(val);
	}
	/**
	* \brief append constraint
	*/
	template<class NAT>
	void appendCSTR(const std::vector<NAT> &val) {
		m_cond->appendCSTR(val);
	}
	/**
	* \brief set constraint
	*/
	template<class UT>
	void setCSTR(const std::vector<UT> &uData) {
		m_cond->setCSTR(uData);
	}
	/**
	* \brief set constraint
	*/
	template<class BT>
	void setCSTR(const std::vector<std::pair<BT, BT>> &bData) {
		m_cond->setCSTR(bData);
	}
	/**
	* \brief set constraint
	*/
	template<class TT>
	void setCSTR(const std::vector<std::tuple<TT, TT, TT>> &tData) {
		m_cond->setCSTR(tData);
	}
	/**
	* \brief set constraint
	*/
	template<class NAT>
	void setCSTR(const std::vector<std::vector<NAT>> &naData) {
		m_cond->setCSTR(naData);
	}
	/**
	* \brief set constraint
	*/
	template<class UT, class BT, class TT, class NAT>
	void setCSTR(
		const std::vector<UT> &uData,
		const std::vector<std::pair<BT, BT>> &bData,
		const std::vector<std::tuple<TT, TT, TT>> &tData,
		const std::vector<std::vector<NAT>> &naData)
	{
		m_cond->setCSTR(uData, bData, tData, naData);
	}
	/**
	* \brief check if node is leaf
	*/
	bool isLeaf() {
		if (m_left == nullptr && m_right == nullptr) {
			return true;
		}
		return false;
	}
	/**
	* \brief get left child
	*/
	Decn_Node *getLeft() {
		return m_left;
	}
	/**
	* \brief set left child
	*/
	void setLeft(Decn_Node *val) {
		m_left = val;
	}
	/**
	* \brief get right child
	*/
	Decn_Node *getRight() {
		return m_right;
	}
	/**
	* \brief set right child
	*/
	void setRight(Decn_Node *val) {
		m_right = val;
	}
	/**
	* \brief get node id
	*/
	unsigned int getID() const {
		return m_id;
	}
	/**
	* \brief set node id
	*/
	void setID(const unsigned int id) {
		m_id = id;
	}
	/**
	* \brief get type of condition
	*/
	TYPE_COND getTypeCond() const {
		return m_type_cond;
	}
	/**
	* \brief set type of condition
	*/
	bool setTypeCond(const TYPE_COND &type_cond) {
		if (type_cond == TYPE_COND::DEFAULT) return false;
		m_type_cond = type_cond;
		return true;
	}
	/**
	* \brief get index of data
	*/
	int getIdxData() const {
		return m_idx_data;
	}
	/**
	* \brief set index of data
	*/
	bool setIdxData(const int idx_data) {
		if (idx_data < 0) return false;
		m_idx_data = idx_data;
		return true;
	}
	/**
	* \brief get index of result
	*/
	int getIdxResult() const {
		return m_idx_result;
	}
	/**
	* \brief set index of result
	*/
	bool setIdxResult(const int idx_result) {
		if (!isLeaf()) return false;
		m_idx_result = idx_result;
		return true;
	}

public:
	Decn_Node *m_left;		/**< left child */
	Decn_Node *m_right;		/**< right child */
	unsigned int m_id;		/**< node id */
	Decn_Cond *m_cond;		/**< condition checking */
	TYPE_COND m_type_cond;	/**< type of condition checking */
	int m_idx_data;			/**< index of data in input doc, where to find data for conditional checking */
	int m_idx_result;		/**< index of result as output, only available for leaf node */
};

/**
* \brief class of binary decision tree
* 
* binary decision tree must be a complete binary tree, the leaf node will output conclusion;
* each non-leaf node contains a condition checking, if true, move to left child, false, move to right child 
*
* \author JING Bichen
*/
class Decn_Tree {
public:
	/**
	* \brief constructor
	*/
	Decn_Tree()
		: m_root(nullptr)
		, m_isTrue2Left(true)
	{

	}
	/**
	* \brief destructor
	*/
	virtual ~Decn_Tree() 
	{
		deleteTree();
	}
	/**
	* \brief destructor
	*/
	void deleteTree() {
		deleteTree(m_root);
	}
	/**
	* \brief check if decision tree is valid
	*/
	bool isValid() {
		// validation
		if (!isComplete()) {
			//DBG_OUT << "Decn_Tree | isValid: not complete\n";
			return false;
		}
		return true;
	}
	/**
	* \brief check if decision tree is complete
	*/
	bool isComplete() {
		return isComplete(m_root);
	}
	/**
	* \brief decision
	* 
	* move to each node for conditional checking, if true, go to left child, else to right;
	* if current node is leaf node, output result index
	*
	*/
	template<class T>
	int decision(const Decn_Doc<T> &data) {
		int result = -1;
		Decn_Node *cur_node = m_root;
		// if condition true, move to left child, default setting
		if (m_isTrue2Left) { 
			while (cur_node != nullptr) {
				// check if current node is leaf
				if (cur_node->isLeaf()) {	// no check for leaf
					result = cur_node->getIdxResult();
					break;
				}
				if (cur_node->check(data)) {
					cur_node = cur_node->m_left;
				}
				else {
					cur_node = cur_node->m_right;
				}
			}
		}
		// if condition true, move to right child
		else {
			while (cur_node != nullptr) {
				// check if current node is leaf
				if (cur_node->isLeaf()) {	// no check for leaf
					result = cur_node->getIdxResult();
					break;
				}
				if (cur_node->check(data)) {
					cur_node = cur_node->m_right;
				}
				else {
					cur_node = cur_node->m_left;
				}
			}
		}
		return result;
	}
protected:
	/**
	* \brief delete subtree at a node, if input root, delete all
	*/
	void deleteTree(Decn_Node *node)
	{
		if (node == nullptr) return;
		deleteTree(node->m_left);
		deleteTree(node->m_right);
		//DBG_OUT << "Deleting node: " << node->m_id;
		delete node;
	}
	/**
	* \brief check if decision tree is complete binary tree
	*/
	bool isComplete(Decn_Node *node) {
		if (node == nullptr) return true;
		// incomplete case, return false
		if (((node->m_left == nullptr) && (node->m_right != nullptr)) ||
			((node->m_left != nullptr) && (node->m_right == nullptr))) 
		{
			return false;
		}
		// check children, need AND operation
		return isComplete(node->m_left) && isComplete(node->m_right);
	}
public:
	/**
	* \brief get root node
	*/
	Decn_Node *getRoot() {
		return m_root;
	}
	/**
	* \brief set root node
	*/
	void setRoot(Decn_Node *val) {
		m_root = val;
	}
	/**
	* \brief is if true, move to left child
	*/
	bool isTrue2Left() {
		return m_isTrue2Left;
	}
	/**
	* \brief set if true move to left or not
	*/
	void setIsTrue2Left(const bool val) {
		m_isTrue2Left = val;
	}
public:
	Decn_Node *m_root;	/**< root node */
	bool m_isTrue2Left;	/**< if true, move to left child, else, if true, move to right */
};

#endif // !_DECN_TREE_H_

//// example: test decision tree
//Decn_Tree *decn_tree = new Decn_Tree();
//Decn_Node *root = new Decn_Node(1);
//root->m_left = new Decn_Node(2);
//root->m_right = new Decn_Node(3);
//root->m_left->m_left = new Decn_Node(4);
//root->m_left->m_right = new Decn_Node(5);
//decn_tree->setRoot(root);
//DBG_OUT << decn_tree->isValid();
//delete decn_tree;
//// test decision tree end