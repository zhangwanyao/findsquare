#ifndef _DECN_COND_H_
#define _DECN_COND_H_
#include <vector>
#include <utility>
#include <tuple>
#include "decn_doc.h"

/**
* \brief class of condition judgement
* \author JING Bichen
*/
class Decn_Cond {
public:
	/**
	* \brief unary condition
	*/
	enum class TYPE_UNARY
	{
		DEFAULT,
		// boolean check
		U_TRUE, U_FALSE,
		// value check
		U_EQUAL, U_LESS, U_OVER, U_N_LESS, U_N_OVER, U_RANGE_IN, U_RANGE_OUT,
		// odd or even
		U_ODD, U_EVEN
	};
	/**
	* \brief binary condition
	*/
	enum class TYPE_BINARY
	{
		DEFAULT,
		// value compare
		B_EQUAL, B_LESS, B_OVER, B_N_LESS, B_N_OVER
	};
	/**
	* \brief ternary condition
	*/
	enum class TYPE_TERNARY
	{
		DEFAULT
	};
	/**
	* \brief narray condition
	*/
	enum class TYPE_NARRAY
	{
		DEFAULT
	};
	/**
	* \brief constraint data type
	*/
	typedef float T_CSTR;
	/**
	* \brief constructor
	*/
	Decn_Cond(const TYPE_UNARY &type_unary) {
		init();
		m_type_unary = type_unary;
	}
	/**
	* \brief constructor
	*/
	Decn_Cond(const TYPE_BINARY &type_binary) {
		init();
		m_type_binary = type_binary;
	}
	/**
	* \brief constructor
	*/
	Decn_Cond(const TYPE_TERNARY &type_ternary) {
		init();
		m_type_ternary = type_ternary;
	}
	/**
	* \brief constructor
	*/
	Decn_Cond(const TYPE_NARRAY &type_narray) {
		init();
		m_type_narray = type_narray;
	}
	/**
	* \brief destructor
	*/
	virtual ~Decn_Cond() {
		clear();
	}
protected:
	/**
	* \brief init
	*/
	void init() {
		m_type_unary = TYPE_UNARY::DEFAULT;
		m_type_binary = TYPE_BINARY::DEFAULT;
		m_type_ternary = TYPE_TERNARY::DEFAULT;
		m_type_narray = TYPE_NARRAY::DEFAULT;
	}
public:
	/**
	* \brief clear
	*/
	void clear() {
		m_cstr.clear();
	}
	/**
	* \brief check unary
	*/
	template<class T>
	bool check(const T &val) {
		switch (m_type_unary)
		{
		case TYPE_UNARY::U_LESS:
			return uIsLess(val);
			break;
		case TYPE_UNARY::U_OVER:
			return uIsOver(val);
			break;
		case TYPE_UNARY::U_N_LESS:
			return uIsNLess(val);
			break;
		case TYPE_UNARY::U_N_OVER:
			return uIsNOver(val);
			break;
		case TYPE_UNARY::U_RANGE_IN:
			return uIsInRange(val);
			break;
		case TYPE_UNARY::U_RANGE_OUT:
			return uIsOutRange(val);
			break;
		default:
			break;
		}
		return false;
	}
	/**
	* \brief check binary
	*/
	template<class T>
	bool check(const std::pair<T, T> &pair) {
		switch (m_type_binary) {
		case TYPE_BINARY::B_EQUAL:
			return bIsEqual(pair);
			break;
		case TYPE_BINARY::B_LESS:
			return bIsLess(pair);
			break;
		case TYPE_BINARY::B_OVER:
			return bIsOver(pair);
			break;
		case TYPE_BINARY::B_N_LESS:
			break;
		case TYPE_BINARY::B_N_OVER:
			break;
		default:
			break;
		}
		return false;
	}
	/**
	* \brief check ternary
	*/
	template<class T>
	bool check(const std::tuple<T, T, T> &val) {
		return false;
	}
	/**
	* \brief check narray
	*/
	template<class T>
	bool check(const std::vector<T> &val) {
		return false;
	}
protected:
	/**
	* \brief check if unary element equal to constraint
	*/
	template<class T>
	bool uIsEqual(const T &val) {
		// data validation
		assert(!m_cstr.m_uData.empty());
		if (val == m_cstr.m_uData[0]) {
			return true;
		}
		return false;
	}
	/**
	* \brief check if unary element less than constraint
	*/
	template<class T>
	bool uIsLess(const T &val) {
		// data validation
		assert(!m_cstr.m_uData.empty());
		if (val < m_cstr.m_uData[0]) {
			return true;
		}
		return false;
	}
	/**
	* \brief check if unary element over constraint
	*/
	template<class T>
	bool uIsOver(const T &val) {
		// data validation
		assert(!m_cstr.m_uData.empty());
		if (val > m_cstr.m_uData[0]) {
			return true;
		}
		return false;
	}
	/**
	* \brief check if unary element no less than constraint
	*/
	template<class T>
	bool uIsNLess(const T &val) {
		// data validation
		assert(!m_cstr.m_uData.empty());
		if (val >= m_cstr.m_uData[0]) {
			return true;
		}
		return false;
	}
	/**
	* \brief check if unary element no over constraint
	*/
	template<class T>
	bool uIsNOver(const T &val) {
		// data validation
		assert(!m_cstr.m_uData.empty());
		if (val <= m_cstr.m_uData[0]) {
			return true;
		}
		return false;
	}
	/**
	* \brief check if unary element in range
	*/
	template<class T>
	bool uIsInRange(const T &val) {
		// data validation
		assert((!m_cstr.m_bData.empty()) || (m_cstr.m_uData.size() > 1));
		T_CSTR lower, upper;
		if (!m_cstr.m_bData.empty()) {
			lower = static_cast<T_CSTR>(m_cstr.m_bData[0].first);
			upper = static_cast<T_CSTR>(m_cstr.m_bData[0].second);
		}
		else {
			lower = static_cast<T_CSTR>(m_cstr.m_uData[0]);
			upper = static_cast<T_CSTR>(m_cstr.m_uData[1]);
		}
		assert(lower <= upper);
		if ((val >= lower) && (val <= upper)) {
			return true;
		}
		return false;
	}
	/**
	* \brief check if unary element out of range
	*/
	template<class T>
	bool uIsOutRange(const T &val) {
		// data validation
		assert((!m_cstr.m_bData.empty()) || (m_cstr.m_uData.size() > 1));
		T_CSTR lower, upper;
		if (!m_cstr.m_bData.empty()) {
			lower = static_cast<T_CSTR>(m_cstr.m_bData[0].first);
			upper = static_cast<T_CSTR>(m_cstr.m_bData[0].second);
		}
		else {
			lower = static_cast<T_CSTR>(m_cstr.m_uData[0]);
			upper = static_cast<T_CSTR>(m_cstr.m_uData[1]);
		}
		assert(lower <= upper);
		if ((val < lower) || (val > upper)) {
			return true;
		}
		return false;
	}
	/**
	* \brief check if binary two elements equal
	*/
	template<class T>
	bool bIsEqual(const std::pair<T, T> &pair) {
		if (pair.first == pair.second) {
			return true;
		}
		return false;
	}
	/**
	* \brief check if binary first element < second element
	*/
	template<class T>
	bool bIsLess(const std::pair<T, T> &pair) {
		if (pair.first < pair.second) {
			return true;
		}
		return false;
	}
	/**
	* \brief check if binary first element > second element
	*/
	template<class T>
	bool bIsOver(const std::pair<T, T> &pair) {
		if (pair.first > pair.second) {
			return true;
		}
		return false;
	}
	/**
	* \brief check if binary first element >= second element
	*/
	template<class T>
	bool bIsNLess(const std::pair<T, T> &pair) {
		if (pair.first >= pair.second) {
			return true;
		}
		return false;
	}
	/**
	* \brief check if binary first element <= second element
	*/
	template<class T>
	bool bIsNOver(const std::pair<T, T> &pair) {
		if (pair.first <= pair.second) {
			return true;
		}
		return false;
	}
public:
	/**
	* \brief get unary type, no set type, force to set according to constructor
	*/
	TYPE_UNARY getTypeUnary() const {
		return m_type_unary;
	}
	/**
	* \brief get binary type, force to set according to constructor
	*/
	TYPE_BINARY getTypeBinary() const {
		return m_type_binary;
	}
	/**
	* \brief get ternary type, force to set according to constructor
	*/
	TYPE_TERNARY getTypeTernary() const {
		return m_type_ternary;
	}
	/**
	* \brief get narray type, force to set according to constructor
	*/
	TYPE_NARRAY getTypeNArray() const {
		return m_type_narray;
	}
	/**
	* \brief get constraint doc
	*/
	Decn_Doc<T_CSTR> getCSTR() const {
		return m_cstr;
	}
	/**
	* \brief set constraint doc
	*/
	template<class UT>
	void setCSTR(const std::vector<UT> &uData) {
		m_cstr.setUData(uData);
	}
	/**
	* \brief set constraint doc
	*/
	template<class BT>
	void setCSTR(const std::vector<std::pair<BT, BT>> &bData) {
		m_cstr.setBData(bData);
	}
	/**
	* \brief set constraint doc
	*/
	template<class TT>
	void setCSTR(const std::vector<std::tuple<TT, TT, TT>> &tData) {
		m_cstr.setTData(tData);
	}
	/**
	* \brief set constraint doc
	*/
	template<class NAT>
	void setCSTR(const std::vector<std::vector<NAT>> &naData) {
		m_cstr.setNAData(naData);
	}
	/**
	* \brief set constraint doc
	*/
	template<class UT, class BT, class TT, class NAT>
	void setCSTR(
		const std::vector<UT> &uData,
		const std::vector<std::pair<BT, BT>> &bData,
		const std::vector<std::tuple<TT, TT, TT>> &tData,
		const std::vector<std::vector<NAT>> &naData)
	{
		m_cstr.setDoc(uData, bData, tData, naData);
	}
	/**
	* \brief append constraint
	*/
	template<class UT>
	void appendCSTR(const UT &val) {
		m_cstr.appendUData(val);
	}
	/**
	* \brief append constraint
	*/
	template<class BT>
	void appendCSTR(const std::pair<BT, BT> &val) {
		m_cstr.appendBData(val);
	}
	/**
	* \brief append constraint
	*/
	template<class TT>
	void appendCSTR(const std::tuple<TT, TT, TT> &val) {
		m_cstr.appendTData(val);
	}
	/**
	* \brief append constraint
	*/
	template<class NAT>
	void appendCSTR(const std::vector<NAT> &val) {
		m_cstr.appedNAData(val);
	}
private:
	TYPE_UNARY			m_type_unary;	/**< type of unary condition */
	TYPE_BINARY			m_type_binary;	/**< type of binary condition */
	TYPE_TERNARY		m_type_ternary;	/**< type of ternary condition */
	TYPE_NARRAY			m_type_narray;	/**< type of narray condition */
	Decn_Doc<T_CSTR>	m_cstr;			/**< constraint doc */
};

#endif // !_DECN_COND_H_

//// example: test decision condition
//Decn_Cond *decn_cond_u = new Decn_Cond(Decn_Cond::TYPE_UNARY::U_RANGE_OUT);
////decn_cond_u->appendCSTR(std::make_pair(1, 2));
//decn_cond_u->appendCSTR(1);
//decn_cond_u->appendCSTR(2);
//
//DBG_OUT << decn_cond_u->check(1.5);
//delete decn_cond_u;
//// test decision condition end
