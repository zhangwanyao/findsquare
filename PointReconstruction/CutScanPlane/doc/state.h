#ifndef _STATE_H_
#define _STATE_H_
#include <string>
#include <array>
#include <math.h>
/**
* \brief base class of document state (multiple)
* \author Bichen JING
*/
class State 
{
public:
	/**
	* \brief constructor
	*/
	State() 
		: m_state(0x00000000)
	{
	}
	/**
	* \brief destructor
	*/
	virtual ~State() {
		clear();
	}
	/**
	* \brief clear
	*/
	virtual void clear() {
	}
	/**
	* \brief check state at location idx (count from right to left, 0-31)
	*/
	bool checkBitState(const unsigned int idx) {
		if (idx > 31) return false;
		unsigned int bitState = pow(2, idx);
		return m_state & bitState;
	}
	/**
	* \brief set state
	*/
	template<class T>
	void setState(const T state) {
		m_state = static_cast<unsigned int>(state);
	}
	/**
	* \brief get state
	*/
	unsigned int getState() const {
		return m_state;
	}
	/**
	* \brief clear state
	*/
	void clearState() {
		m_state = 0;
	}
	/**
	* \brief append state
	*/
	template<class T>
	void appendState(const T state) {
		m_state |= static_cast<unsigned int>(state);
	}
	/**
	* \brief remove state
	*/
	template<class T>
	void removeState(const T state) {
		m_state &= ~static_cast<unsigned int>(state);
	}
	/**
	* \brief check if two states equal or not
	*/
	template<class T>
	bool isEqual(const T state) {
		return (m_state == static_cast<unsigned int>(state));
	}
	/**
	* \brief operation AND
	*/
	template<class T>
	unsigned int opAND(const T state) {
		return m_state & static_cast<unsigned int>(state);
	}
	/**
	* \brief operation OR
	*/
	template<class T>
	unsigned int opOR(const T state) {
		return m_state | static_cast<unsigned int>(state);
	}
	/**
	* \brief operation XOR
	*/
	template<class T>
	unsigned int opXOR(const T state) {
		return m_state ^ static_cast<unsigned int>(state);
	}

protected:
	unsigned int m_state;	/**< state */
};

#endif // !_STATE_H_
