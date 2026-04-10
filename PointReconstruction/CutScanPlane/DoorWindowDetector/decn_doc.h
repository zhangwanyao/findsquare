#ifndef _DECN_DOC_H_
#define _DECN_DOC_H_
#include <vector>
#include <utility>
#include <tuple>
#include <opencv2\core.hpp>
/**
* \brief class of decision document/data
* \author JING Bichen
*/
template<class DT>	/**< data typename/template */
class Decn_Doc {
public:
	typedef std::vector<DT> UData;
	typedef std::vector<std::pair<DT, DT>> BData;
	typedef std::vector<std::tuple<DT, DT, DT>> TData;
	typedef std::vector<std::vector<DT>> NAData;
	/**
	* \brief constructor
	*/
	Decn_Doc() 
	{
	}
	/**
	* \brief destructor
	*/
	virtual ~Decn_Doc() 
	{
		clear();
	}
	/**
	* \brief clear
	*/
	void clear() 
	{
		m_uData.clear();
		m_bData.clear();
		m_tData.clear();
		clearNAData();
	}
	/**
	* \brief append uData
	*/
	template<class T>
	void appendUData(const T &val) {
		m_uData.push_back(static_cast<DT>(val));
	}
	/**
	* \brief append bData
	*/
	template<class T>
	void appendBData(const std::pair<T, T> &val) {
		m_bData.push_back(std::make_pair(
			static_cast<DT>(val.first), static_cast<DT>(val.second)));
	}
	/**
	* \brief append tData
	*/
	template<class T>
	void appendTData(const std::tuple<T, T, T> &val) {
		m_tData.push_back(std::make_tuple(
			static_cast<DT>(std::get<0>(val)), 
			static_cast<DT>(std::get<1>(val)), 
			static_cast<DT>(std::get<2>(val))));
		return;
	}
	/**
	* \brief append naData
	*/
	template<class T>
	bool appedNAData(const std::vector<T> &val) {
		// data validation
		if (val.empty()) return false;
		size_t n_val = val.size();
		std::vector<DT> temp;
		temp.resize(n_val);
		for (size_t i = 0; i < n_val; i++) {
			temp[i] = static_cast<DT>(val[i]);
		}
		m_naData.push_back(temp);
		temp.clear();
		return true;
	}
	/**
	* \brief set doc
	*/
	template<class UT, class BT, class TT, class NAT>
	void setDoc(
		const std::vector<UT> &uData, 
		const std::vector<std::pair<BT, BT>> &bData,
		const std::vector<std::tuple<TT, TT, TT>> &tData,
		const std::vector<std::vector<NAT>> &naData)
	{
		setUData(uData);
		setBData(bData);
		setTData(tData);
		setNAData(naData);
	}
	/**
	* \brief set uData
	*/
	template<class T>
	void setUData(const std::vector<T> &input) {
		if (input.empty()) return;
		size_t n_val = input.size();
		m_uData.resize(n_val);
		for (size_t i = 0; i < n_val; i++) {
			m_uData[i] = static_cast<DT>(input[i]);
		}
	}
	/**
	* \brief set bData
	*/
	template<class T>
	void setBData(const std::vector<std::pair<T, T>> &input) {
		if (input.empty()) return;
		size_t n_val = input.size();
		m_bData.resize(n_val);
		for (size_t i = 0; i < n_val; i++) {
			m_bData[i] = std::make_pair(
				static_cast<DT>(input[i].first), 
				static_cast<DT>(input[i].second));
		}
	}
	/**
	* \brief set tData
	*/
	template<class T>
	void setTData(const std::vector<std::tuple<T, T, T>> &input) {
		if (input.empty()) return;
		size_t n_val = input.size();
		m_tData.resize(n_val);
		for (size_t i = 0; i < n_val; i++) {
			m_tData[i] = std::make_tuple(
				static_cast<DT>(std::get<0>(input[i])),
				static_cast<DT>(std::get<1>(input[i])),
				static_cast<DT>(std::get<2>(input[i]))
			);
		}
	}
	/**
	* \brief set naData
	*/
	template<class T>
	void setNAData(const std::vector<std::vector<T>> &input) {
		if (input.empty()) return;
		clearNAData();
		size_t n_out = input.size();
		for (size_t i_out = 0; i_out < n_out; i_out++) {
			size_t n_in = input[i_out].size();
			std::vector<DT> temp;
			temp.resize(n_in);
			for (size_t i_in = 0; i_in < n_in; i_in++) {
				temp[i_in] = static_cast<DT>(input[i_out][i_in]);
			}
			m_naData.push_back(temp);
			temp.clear();
		}
	}
	/**
	* \brief get uData
	*/
	template<class T>
	void getUData(std::vector<T> &output) {
		size_t n_size = m_uData.size();
		output.resize(n_size);
		for (size_t i = 0; i < n_size; i++) {
			output[i] = static_cast<T>(m_uData[i]);
		}
	}
	/**
	* \brief get bData
	*/
	template<class T>
	void getBData(std::vector<std::pair<T, T>> &output) {
		size_t n_size = m_bData.size();
		output.resize(n_size);
		for (size_t i = 0; i < n_size; i++) {
			output[i] = std::make_pair(static_cast<T>(m_bData[i].first), static_cast<T>(m_bData[i].second));
		}
	}
	/**
	* \brief get tData
	*/
	template<class T>
	void getTData(std::vector<std::tuple<T, T, T>> &output) {
		size_t n_size = m_bData.size();
		output.resize(n_size);
		for (size_t i = 0; i < n_size; i++) {
			output[i] = std::make_tuple(
				static_cast<T>(std::get<0>(m_tData[i])), 
				static_cast<T>(std::get<1>(m_tData[i])),
				static_cast<T>(std::get<2>(m_tData[i]))
			);
		}
	}
	/**
	* \brief get naData
	*/
	template<class T>
	void getNAData(std::vector<std::vector<T>> &output) {
		// clear output
		size_t n_prev = output.size();
		if (n_prev) {
			for (size_t i = 0; i < n_prev; i++) {
				output[i].clear();
			}
		}
		output.clear();
		// append temp vector to output
		size_t out_size = m_naData.size();
		for (size_t i = 0; i < out_size; i++) {
			size_t in_size = m_naData[i].size();
			std::vector<T> temp;
			temp.resize(in_size);
			for (size_t j = 0; j < in_size; j++) {
				temp[i] = static_cast<T>(m_naData[i][j]);
			}
			output.push_back(temp);
		}
	}
protected:
	/**
	* \brief clear NAData
	*/
	void clearNAData() {
		for (typename NAData::iterator it = m_naData.begin(); it != m_naData.end(); it++) {
			it->clear();
		}
		m_naData.clear();
	}
public:
	UData m_uData;		/**< unary data */
	BData m_bData;		/**< binary data */
	TData m_tData;		/**< ternary data */
	NAData m_naData;	/**< nArray data */
};

#endif // !_DECN_DOC_H_

//// example
//Decn_Doc<float> *doc = new Decn_Doc<float>();
//
//std::vector<int> uData;
//uData.push_back(1);
//
//std::vector<std::pair<double, double>> bData;
//bData.push_back(std::make_pair(2, 3));
//
//std::vector<std::tuple<int, int, int>> tData;
//tData.push_back(std::make_tuple(4, 5, 6));
//
//std::vector<std::vector<float>> naData;
//std::vector<float> temp;
//temp.push_back(1);
//naData.push_back(temp);
//
//doc->setUData(uData);
//doc->setBData(bData);
//doc->setTData(tData);
//doc->setNAData(naData);
//
//DBG_OUT << doc->m_uData[0];
//DBG_OUT << doc->m_bData[0].first << doc->m_bData[0].second;
//DBG_OUT << std::get<0>(doc->m_tData[0])
//<< std::get<1>(doc->m_tData[0])
//<< std::get<2>(doc->m_tData[0]);
//
//DBG_OUT << doc->m_naData[0][0];
//
//delete doc;
