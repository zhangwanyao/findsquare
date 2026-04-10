#ifndef _UTIL_SAMPLER_1D_H_
#define _UTIL_SAMPLER_1D_H_
#include <vector>
#include <limits>
#include <utility>
#include <algorithm>
//#include "../config_dbg.h"
/**
* \brief 1D sampler
*
* compute samples from a given value range with margin and sampling requirement.
* sampling is classified as uniform, non-uniform with ratio and non-uniform with length, three types
* for non-uniform sampling, classified as asymmetric, symmetric with odd number of samples, and with even number of samples.
* 
* illustration, for a given value range defined by [val_lower, val_upper],
* target sampling range is constrained by maring_lower and margin_upper,
* namely, sampling in range [val_lower + margin_lower, val_upper - margin_upper]
* ||(val_lower)----------------------------------------------------------------(val_upper)||
* ||--margin_lower--||(sample_0)-------|------|------|------(sample_n-1)||--margin_upper--||
*
* sampling result must contain the two ending/bounding value, namely val_lower + margin_lower and val_upper - margin_upper
* for n samples, number of inner samples is n - 2, there are n - 1 segments separated by inner samples
* for asymmetric sampling, ratio and length vector contain the first (n - 2) segment ratio, the last ratio is got by deduction from 1
* for symmetric sampling, ratio and length vector contain the value in [0, 0.5] part of value range,
* if sampling number is odd, namely contains middle value, ratio and length vector contain (n - 1) / 2 - 1 elements/segments
* if sampling number is even, the number of elements is n / 2 - 1
*
* \author JING Bichen
*/

template<class ST>
class Util_Sampler_1D {
public:
	//typedef float ST;	/**< scalar typename/template */
	/**
	* \brief division condition
	*/
	enum class COND_DIV {
		UNIFORM,
		NON_UNIFORM_RATIO,
		NON_UNIFORM_LEN
	};
	/**
	* \brief symmetrical condition, only for non-uniform
	*/
	enum class COND_SYM {
		ASYMMETRIC,
		SYMMETRIC_ODD,	// sample mid value
		SYMMETRIC_EVEN,	// not sample mid value
	};
	/**
	* \brief constructor
	*/
	Util_Sampler_1D()
		: m_val_lower(std::numeric_limits<ST>::infinity())
		, m_val_upper(-std::numeric_limits<ST>::infinity())
		, m_margin_lower(static_cast<ST>(0))
		, m_margin_upper(static_cast<ST>(0))
		, m_nSample(0)
		, m_cond_div(COND_DIV::UNIFORM)
		, m_cond_sym(COND_SYM::ASYMMETRIC)
	{

	}
	/**
	* \brief destructor
	*/
	virtual ~Util_Sampler_1D() {
		clear();
	}
	/**
	* \brief clear
	*/
	virtual void clear() {
		m_samples.clear();
		m_intvl_ratio.clear();
		m_intvl_len.clear();
	}
	/**
	* \brief sample
	*/
	virtual bool sample() {
		// data validation
		if (!isVariableValid()) return false;
		switch (m_cond_div)
		{
		case COND_DIV::UNIFORM:
			calSample_uniform();
			break;
		case COND_DIV::NON_UNIFORM_RATIO:
			calSample_nonunf_ratio();
			break;
		case COND_DIV::NON_UNIFORM_LEN:
			calSample_nonunf_len();
			break;
		default:
			break;
		}
		return true;
	}
	/**
	* \brief sample uniform
	*/
	template<class T>
	bool sample_uniform(const std::pair<T, T> &bounds, const T margin_lower, const T margin_upper, const unsigned int nSample) 
	{
		// set variables
		m_cond_div = COND_DIV::UNIFORM;
		if (!setBounds(bounds.first, bounds.second)) {
			//DBG_OUT << "error: util_sampler_1d | set bounds\n";
			return false;
		}
		if (!setMargins(margin_lower, margin_upper)) {
			//DBG_OUT << "error: util_sampler_1d | set margin\n";
			return false;
		}
		if (!setNSample(nSample)) {
			//DBG_OUT << "error: util_sampler_1d | set number of sample\n";
			return false;
		}
		return sample();
	}

	/**
	* \brief sample uniform
	*/
	template<class T>
	bool sample_uniform(const std::pair<T, T> &bounds, const T margin_lower, const T margin_upper, const unsigned int nSample, 
		std::vector<T> &output)
	{
		bool isSucceed = sample_uniform(bounds, margin_lower, margin_upper, nSample);
		if (isSucceed) getSamples(output);
		return isSucceed;
	}
	/**
	* \brief sample non-uniform with interval ratio
	*/
	template<class T>
	bool sample_nonunf_ratio(const std::pair<T, T> &bounds, const T margin_lower, const T margin_upper, 
		const COND_SYM cond_sym, const std::vector<T> &intvl_ratio) 
	{
		// set variable
		m_cond_div = COND_DIV::NON_UNIFORM_RATIO;
		m_cond_sym = cond_sym;
		if (!setBounds(bounds.first, bounds.second)) {
			//DBG_OUT << "error: util_sampler_1d | set bounds\n";
			return false;
		}
		if (!setMargins(margin_lower, margin_upper)) {
			//DBG_OUT << "error: util_sampler_1d | set margin\n";
			return false;
		}
		if (!setIntvl_ratio(intvl_ratio)) {
			//DBG_OUT << "error: util_sampler_1d | set interval ratio\n";
			return false;
		}
		return sample();
	}
	/**
	* \brief sample non-uniform with interval ratio
	*/
	template<class T>
	bool sample_nonunf_ratio(const std::pair<T, T> &bounds, const T margin_lower, const T margin_upper,
		const COND_SYM cond_sym, const std::vector<T> &intvl_ratio, std::vector<T> &output)
	{
		bool isSucceed = sample_nonunf_ratio(bounds, margin_lower, margin_upper, cond_sym, intvl_ratio);
		if (isSucceed) getSamples(output);
		return isSucceed;
	}
	/**
	* \brief sample non-uniform with interval length
	*/
	template<class T>
	bool sample_nonunf_len(const std::pair<T, T> &bounds, const T margin_lower, const T margin_upper,
		const COND_SYM cond_sym, const std::vector<T> &intvl_len)
	{
		// set variable
		m_cond_div = COND_DIV::NON_UNIFORM_LEN;
		m_cond_sym = cond_sym;
		if (!setBounds(bounds.first, bounds.second)) {
			//DBG_OUT << "error: util_sampler_1d | set bounds\n";
			return false;
		}
		if (!setMargins(margin_lower, margin_upper)) {
			//DBG_OUT << "error: util_sampler_1d | set margin\n";
			return false;
		}
		if (!setIntvl_len(intvl_len)) {
			//DBG_OUT << "error: util_sampler_1d | set interval ratio\n";
			return false;
		}
		return sample();
	}
	/**
	* \brief sample non-uniform with interval length
	*/
	template<class T>
	bool sample_nonunf_len(const std::pair<T, T> &bounds, const T margin_lower, const T margin_upper,
		const COND_SYM cond_sym, const std::vector<T> &intvl_len, std::vector<T> &output)
	{
		bool isSucceed = sample_nonunf_len(bounds, margin_lower, margin_upper, cond_sym, intvl_len);
		if (isSucceed) getSamples(output);
		return isSucceed;
	}
protected:
	/**
	* \brief check member variable validation
	*/
	virtual bool isVariableValid() {
		if (m_val_lower >= m_val_upper) {
			//DBG_OUT << "error: util_sampler_1d | data range invalid\n";
			return false;
		}
		if (m_margin_lower < 0 || m_margin_upper < 0) {
			//DBG_OUT << "error: util_sampler_1d | margin < 0\n";
			return false;
		}
		if (m_margin_lower + m_margin_upper >= m_val_upper - m_val_lower) {
			//DBG_OUT << "error: util_sampler_1d | margin > data range\n";
			return false;
		}
		ST sum;
		switch (m_cond_div) {
		case COND_DIV::UNIFORM:
			if (m_nSample < 1) {
				//DBG_OUT << "error: util_sampler_1d | number of sampling = 0\n";
				return false;
			}
			break;
		case COND_DIV::NON_UNIFORM_RATIO:
			if (m_intvl_ratio.size() < 1) {
				//DBG_OUT << "error: util_sampler_1d | invalid number of interval ratio\n";
				return false;
			}
			sum = static_cast<ST>(0);
			for (unsigned int i = 0; i < m_intvl_ratio.size(); i++) {
				if (m_intvl_ratio[i] <= 0) {
					//DBG_OUT << "error: util_sampler_1d | invalid ratio value\n";
					return false;
				}
				sum += m_intvl_ratio[i];
			}
			if (m_cond_sym == COND_SYM::ASYMMETRIC) {
				if (sum >= static_cast<ST>(1)) { // if == 1, last position re-compute
					//DBG_OUT << "error: util_sampler_1d | invalid ratio summation\n";
					return false;
				}
			}
			if (m_cond_sym == COND_SYM::SYMMETRIC_ODD || m_cond_sym == COND_SYM::SYMMETRIC_EVEN) { 
				// symmetric only needs half ratio
				if (sum >= static_cast<ST>(0.5)) { // if == 0.5, mid position re-compute
					//DBG_OUT << "error: util_sampler_1d | invalid ratio summation\n";
					return false;
				}
			}
			break;
		case COND_DIV::NON_UNIFORM_LEN:
			if (m_intvl_len.size() < 1) {
				//DBG_OUT << "error: util_sampler_1d | invalid number of interval length\n";
				return false;
			}
			sum = static_cast<ST>(0);
			for (unsigned int i = 0; i < m_intvl_len.size(); i++) {
				if (m_intvl_len[i] <= 0) {
					//DBG_OUT << "error: util_sampler_1d | invalid length value\n";
					return false;
				}
				sum += m_intvl_len[i];
			}
			if (m_cond_sym == COND_SYM::ASYMMETRIC) {
				// if equal, last position re-compute
				if (sum >= (m_val_upper - m_val_lower - m_margin_lower - m_margin_upper)) { 
					//DBG_OUT << "error: util_sampler_1d | length summation > target segment\n";
					return false;
				}
			}
			if (m_cond_sym == COND_SYM::SYMMETRIC_ODD || m_cond_sym == COND_SYM::SYMMETRIC_EVEN) {
				// if equal, mid position re-compute
				if (sum >= ((m_val_upper - m_val_lower - m_margin_lower - m_margin_upper) * 0.5)) {
					//DBG_OUT << "error: util_sampler_1d | length summation > target segment\n";
					return false;
				}
			}
			break;
		default:
			break;
		}
		return true;
	}

	/**
	* \brief compute sample, uniform
	*
	* nSample == 1, return middle value 
	* nSample > 1, return two ending and internal positions
	*
	*/
	void calSample_uniform() {
		m_samples.clear();
		// update lower and upper based on margin
		ST lower = m_val_lower + m_margin_lower;
		ST upper = m_val_upper - m_margin_upper;
		if (m_nSample == 1) // return middle value
		{
			m_samples.push_back((lower + upper) * 0.5);
			return;
		}
		ST intvl_len = (upper - lower) / (m_nSample - 1);
		for (unsigned int i = 0; i < m_nSample; i++) {
			m_samples.push_back(lower + i * intvl_len);
		}
	}
	/**
	* \brief compute sample, non_uniform, use ratio
	*/
	void calSample_nonunf_ratio() {
		m_samples.clear();
		// update lower and upper based on margin
		ST lower = m_val_lower + m_margin_lower;
		ST upper = m_val_upper - m_margin_upper;
		ST range = upper - lower;
		ST sum = static_cast<ST>(0);
		switch (m_cond_sym) {
		case COND_SYM::ASYMMETRIC:
			m_samples.push_back(lower);
			for (unsigned int i = 0; i < m_intvl_ratio.size(); i++) {
				sum += m_intvl_ratio[i] * range;
				m_samples.push_back(lower + sum);
			}
			m_samples.push_back(upper);
			break;
		case COND_SYM::SYMMETRIC_ODD:
			m_samples.push_back(lower);
			for (unsigned int i = 0; i < m_intvl_ratio.size(); i++) {
				sum += m_intvl_ratio[i] * range;
				m_samples.push_back(lower + sum);
				m_samples.push_back(upper - sum);
			}
			m_samples.push_back(static_cast<ST>((upper + lower) * 0.5));
			m_samples.push_back(upper);
			break;
		case COND_SYM::SYMMETRIC_EVEN:
			m_samples.push_back(lower);
			for (unsigned int i = 0; i < m_intvl_ratio.size(); i++) {
				sum += m_intvl_ratio[i] * range;
				m_samples.push_back(lower + sum);
				m_samples.push_back(upper - sum);
			}
			m_samples.push_back(upper);
			break;
		default:
			break;
		}
		sortSamples();
	}
	/**
	* \brief compute sample, non_uniform, use length
	*/
	void calSample_nonunf_len() {
		m_samples.clear();
		// update lower and upper based on margin
		ST lower = m_val_lower + m_margin_lower;
		ST upper = m_val_upper - m_margin_upper;
		ST range = upper - lower;
		ST sum = static_cast<ST>(0);
		switch (m_cond_sym) {
		case COND_SYM::ASYMMETRIC:
			m_samples.push_back(lower);
			for (unsigned int i = 0; i < m_intvl_len.size(); i++) {
				sum += m_intvl_len[i];
				m_samples.push_back(lower + sum);
			}
			m_samples.push_back(upper);
			break;
		case COND_SYM::SYMMETRIC_ODD:
			m_samples.push_back(lower);
			for (unsigned int i = 0; i < m_intvl_len.size(); i++) {
				sum += m_intvl_len[i];
				m_samples.push_back(lower + sum);
				m_samples.push_back(upper - sum);
			}
			m_samples.push_back(static_cast<ST>((upper + lower) * 0.5));
			m_samples.push_back(upper);
			break;
		case COND_SYM::SYMMETRIC_EVEN:
			m_samples.push_back(lower);
			for (unsigned int i = 0; i < m_intvl_len.size(); i++) {
				sum += m_intvl_len[i];
				m_samples.push_back(lower + sum);
				m_samples.push_back(upper - sum);
			}
			m_samples.push_back(upper);
			break;
		default:
			break;
		}
		sortSamples();
	}
	/**
	* \brief sort samples in ascending order
	*/
	virtual void sortSamples() {
		std::sort(m_samples.begin(), m_samples.end());
	}

public:
	/**
	* \brief set lower and upper bound value of data range
	*/
	template<class T>
	bool setBounds(const T lower, const T upper) {
		if (lower >= upper) return false; // data validation
		m_val_lower = static_cast<ST>(lower);
		m_val_upper = static_cast<ST>(upper);
		return true;
	}
	/**
	* \brief get lower and upper bound value of data range
	*/
	template<class T>
	void getBounds(T &lower, T &upper) {
		lower = static_cast<T>(m_val_lower);
		upper = static_cast<T>(m_val_upper);
	}
	/**
	* \brief set lower and upper margin value
	*/
	template<class T>
	bool setMargins(const T margin_lower, const T margin_upper) {
		if (margin_lower < 0 || margin_upper < 0) return false;
		m_margin_lower = static_cast<ST>(margin_lower);
		m_margin_upper = static_cast<ST>(margin_upper);
		return true;
	}
	/**
	* \brief get lower and upper margin value
	*/
	template<class T>
	void getMargins(T &margin_lower, T &margin_upper) {
		margin_lower = static_cast<T>(m_margin_lower);
		margin_upper = static_cast<T>(m_margin_upper);
	}
	/**
	* \brief set number of sampling
	*/
	bool setNSample(const unsigned int n) {
		if (n < 1) return false;
		m_nSample = n;
		return true;
	}
	/**
	* \brief get number of sampling
	*/
	unsigned int getNSample() const {
		return m_nSample;
	}
	/**
	* \brief get samples
	*/
	template<class T>
	void getSamples(std::vector<T> &output) {
		size_t n_sample = m_samples.size();
		output.resize(n_sample);
		for (size_t i = 0; i < n_sample; i++) {
			output[i] = static_cast<T>(m_samples[i]);
		}
	}
	/**
	* \brief set division condition
	*/
	void setCond_div(const COND_DIV val) {
		m_cond_div = val;
	}
	/**
	* \brief get division condition
	*/
	COND_DIV getCond_div() const {
		return m_cond_div;
	}
	/**
	* \brief set symmetric condition
	*/
	void setCond_sym(const COND_SYM val) {
		m_cond_sym = val;
	}
	/**
	* \brief get symmetric condition
	*/
	COND_SYM getCond_sym() const {
		return m_cond_sym;
	}
	/**
	* \brief set interval ratio
	*/
	template<class T>
	bool setIntvl_ratio(const std::vector<T> &vals) {
		size_t n_val = vals.size();
		// data validation
		if (n_val < 1) return false; // number of value
		for (size_t i = 0; i < n_val; i++) {
			if (vals[i] <= static_cast<T>(0)) return false;	// invalid ratio
		}
		// check ratio summation validation in sampling function, other data may not set here
		// copy data
		m_intvl_ratio.resize(n_val);
		for (unsigned int i = 0; i < n_val; i++) {
			m_intvl_ratio[i] = static_cast<ST>(vals[i]);
		}
		return true;
	}
	/**
	* \brief set interval length
	*/
	template<class T>
	bool setIntvl_len(const std::vector<T> &vals) {
		size_t n_val = vals.size();
		// data validation
		if (n_val < 1) return false; // number of value
		for (size_t i = 0; i < n_val; i++) {
			if (vals[i] <= static_cast<T>(0)) return false;	// invalid ratio
		}
		// check length summation validation in sampling function, other data may not set here
		// copy data
		m_intvl_len.resize(n_val);
		for (unsigned int i = 0; i < n_val; i++) {
			m_intvl_len[i] = static_cast<ST>(vals[i]);
		}
		return true;
	}

private:
	ST m_val_lower;					/**< lower bound value of data range */
	ST m_val_upper;					/**< upper bound value of data range */
	ST m_margin_lower;				/**< lower margin value */
	ST m_margin_upper;				/**< upper margin value */
	unsigned int m_nSample;			/**< number of sample */
	std::vector<ST> m_samples;		/**< sampled values */
	COND_DIV m_cond_div;			/**< division condition, is sampling uniform, uniform must be symmetric */
	COND_SYM m_cond_sym;			/**< symmetric condition, is sampling symmetric, for non-uniform */
	std::vector<ST> m_intvl_ratio;	/**< interval ratio, from lower side to upper side */
	std::vector<ST> m_intvl_len;	/**< interval length, from lower side to upper side */
};

#endif // !_UTIL_SAMPLER_1D_H_

//// example: test sampler ..
//Util_Sampler_1D<float> *sampler = new Util_Sampler_1D<float>();
//std::vector<float> samples;
//float lower = 1.f;
//float upper = 2.f;
//float margin_lower = 0.f;
//float margin_upper = 0.f;

//std::vector<float> intvl_ratio;
//intvl_ratio.push_back(0.1);
////intvl_ratio.push_back(0.8);

//std::vector<float> intvl_len;
//intvl_len.push_back(0.1);
//intvl_len.push_back(0.2);
//intvl_len.push_back(0.3);

//bool isSucceed = false;

//// test uniform
////sampler->sample_uniform(std::make_pair(lower, upper), margin_lower, margin_upper, 0);
////sampler->sample_uniform(std::make_pair(1.f, 1.f), margin_lower, margin_upper, 5);
////sampler->sample_uniform(std::make_pair(1.f, 0.f), margin_lower, margin_upper, 5);
////sampler->sample_uniform(std::make_pair(lower, upper), -1.f, margin_upper, 5);
////sampler->sample_uniform(std::make_pair(lower, upper), 0.6f, 0.6f, 5);

////bool isSucceed = sampler->sample_uniform(std::make_pair(lower, upper), margin_lower, margin_upper, 5, samples);

//// test non-uniform
//isSucceed = sampler->sample_nonunf_ratio(std::make_pair(lower, upper), margin_lower, margin_upper, Util_Sampler_1D<float>::COND_SYM::SYMMETRIC_ODD, intvl_ratio, samples);
//
//isSucceed = sampler->sample_nonunf_len(std::make_pair(lower, upper), margin_lower, margin_upper, Util_Sampler_1D<float>::COND_SYM::ASYMMETRIC, intvl_len, samples);

//if (isSucceed) {
//	for (unsigned int i = 0; i < samples.size(); i++) {
//		DBG_OUT << samples[i];
//	}
//}

//samples.clear();
//delete sampler;
//// test sampler end
