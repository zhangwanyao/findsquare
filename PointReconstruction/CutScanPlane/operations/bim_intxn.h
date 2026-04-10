#ifndef _BIM_INTXN_H_
#define _BIM_INTXN_H_
#include <vector>
#include <utility>
/**
* \brief class of BIM intersection
*/
class BIM_Intxn {
public:
	/**
	* \brief constructor
	*/
	BIM_Intxn() {

	}
	/**
	* \brief destructor
	*/
	~BIM_Intxn() {

	}
	/**
	* \brief check if idx in idx list
	*/
	template<typename T0, typename T1>
	static bool isIdxInList(const T0 idx, const std::vector<T1> &list) {
		return std::find(list.begin(), list.end(), static_cast<T1>(idx)) != list.end();
	}
	/**
	* \brief compute intersection
	*/
	template<class T1, class T2, typename S>
	static bool isIntxn(T1 *doc_1, T2 *doc_2, const S dilate)
	{
		// data validation
		if (doc_1 == nullptr || doc_2 == nullptr) return false;
		S d = MAX(dilate, 0);
		BBox3D b1 = doc_1->getBBox();
		BBox3D b2 = doc_2->getBBox();
		b1.dilate(static_cast<float>(d));
		b2.dilate(static_cast<float>(d));
		return b1.isOverlap(b2);
	}
	/**
	* \brief compute intersection
	* \param docs docs holding data
	* \param idx_proc interested idx of docs to check intersection
	* \param dilate dilate factor
	* \param intxn output intersection pairs
	*/
	template<class T>
	static void compute(const std::vector<T*> &docs,
		const std::vector<unsigned int> &idx_proc, const float dilate,
		std::vector<std::pair<unsigned int, unsigned int>> &intxn)
	{
		typedef std::pair<unsigned int, unsigned int> pairID;
		if (idx_proc.empty()) return;
		intxn.clear();
		// loop docs
		for (unsigned int i = 0; i < docs.size(); i++) {
			if (!BIM_Intxn::isIdxInList(docs[i]->getIdxPlane(), idx_proc)) continue;
			T *doc_i = docs[i];
			if (doc_i == nullptr) continue;
			// check overlap between bbox
			for (unsigned int j = i + 1; j < docs.size(); j++) {
				if (!BIM_Intxn::isIdxInList(docs[j]->getIdxPlane(), idx_proc)) continue;
				T *doc_j = docs[j];
				if (doc_j == nullptr) continue;
				// check intersection
				if (BIM_Intxn::isIntxn(doc_i, doc_j, dilate)) {
					// if overlap, add to intersection table
					pairID temp = std::make_pair(MIN(i, j), MAX(i, j));
					// check if exist
					bool isExist = false;
					for (size_t i = 0; i < intxn.size(); i++) {
						if (temp == intxn[i]) {
							isExist = true;
							break;
						}
					}
					if (!isExist) {
						intxn.push_back(temp);
					}
				}
			}
		}
	}
};

#endif // !_BIM_INTXN_H_
