#pragma once
template<class T>
struct Tree_Node {
	T  _val;
	vector<Tree_Node<T>*> _children;

	//Node() {}

	//Node(const T& val, const vector<Tree_Node<T>*>& children) {
	//	_val = val;
	//	_children = children;
	//};
	Tree_Node(const T& val, const vector<Tree_Node<T>*>* children) {
			_val = val;
			if (children == NULL)
				return;
			_children = *children;
	};
};
template <class T>
class Tree
{
public:
	Tree()
		:_root(NULL)
	{}
	Tree(T child)
		:_root(NULL)
	{_root = new Tree_Node<T>(child, NULL);}
	void clear();
	Tree_Node<T>* Find(T value);
	bool Add_Node(T parent, T child);
	void Clear_Node();
	int size();
	int Height();
	int Leaves();
	bool empty();
	Tree_Node<T>* get()
	{
		return _root;
	}
private:
	Tree_Node<T> *Find_R(Tree_Node<T>*root, T value);
	int Height_R(Tree_Node<T>* root);
	int Size_R(Tree_Node<T>* root);
	int Leaves_R(Tree_Node<T>* root);
private:
	Tree_Node<T>* _root;
};


template <class T>
void Tree<T>::Clear_Node()
{
	if (_root == NULL)
		return;

	for (auto &child : _root->_children)
	{
		delete child;
	}
	_root->_children.resize(0);
}
template <class T>
bool Tree<T>::Add_Node(T parent, T child)
{
	if (_root == NULL)
	{  /*设置为根节点*/
		_root = new Tree_Node<T>(child, NULL);
		return true;
	}

	/*寻找值 = parent的节点*/
	Tree_Node<T>* cur = Find(parent);
	Tree_Node<T>* child_ = new Tree_Node<T>(child, NULL);

	cur->_children.push_back(child_);
}
template <class T>
Tree_Node<T>* Tree<T>::Find(T value)
{
	return Find_R(_root, value);
}

template <class T>
Tree_Node<T>* Tree<T>::Find_R(Tree_Node<T>* root, T value)
{
	if (root->_val == value)
		return root;
	for (auto &child : root->_children)
	{
		Find_R(child, value);
	}
	return NULL;
}
template <class T>
int Tree<T>::Height()
{
	Height_R(_root);
}

template <class T>
int Tree<T>::Height_R(Tree_Node<T>* root)
{
	if (root == NULL)
		return 0;
	int height = 0;

	for (auto &child : root->_children)
	{
		height = max(height, Height_R(child));
	}
	return height + 1;
}

template <class T>
int Tree<T>::size()
{
	return Size_R(_root);
}

template <class T>
int Tree<T>::Size_R(Tree_Node<T>* root)
{
	if (root == NULL)
		return 0;
	int size = 0;
	for (auto &child : root->_children)
	{
		size += Size_R(child);
	}
	return size + 1;
}

template <class T>
bool Tree<T>::empty()
{
	if (_root == NULL)
		return true;
	else
		return false;
}

template <class T>
int Tree<T>::Leaves()
{
	return Leaves_R(_root);
}

template <class T>
int Tree<T>::Leaves_R(Tree_Node<T>* root)
{
	if (root == NULL)
		return 0;
	if (root->_children.size() == 0)
		return 1;

	int leaves = 0;

	for (auto &child : root->_children)
	{
		leaves += Leaves_R(child);
	}
}