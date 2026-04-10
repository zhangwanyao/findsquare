#ifndef _STATE_BIM_H_
#define _STATE_BIM_H_
#include "state.h"

/**
* \brief enum class attribute of bim plane
*/
enum class ATTR_BIM_PLANE
{
	NOTVALID = 0,
	VALID = 1 << 0,
	HORIZ = 1 << 1,
	VERT = 1 << 2,
	FLOOR = 1 << 3,
	CEILING = 1 << 4,
	WALL = 1 << 5,
	BEAM = 1 << 6
};

/**
* \brief enum class operator and
*/
inline ATTR_BIM_PLANE operator | (ATTR_BIM_PLANE a, ATTR_BIM_PLANE b) {
	return static_cast<ATTR_BIM_PLANE>(
		static_cast<unsigned int>(a) | static_cast<unsigned int>(b));
}

/**
* \brief state of bim plane
*/
class State_BIM_Plane : public State {
public:
	/**
	* \brief constructor
	*/
	State_BIM_Plane() {
	}
	/**
	* \brief destructor
	*/
	~State_BIM_Plane() {
		clear();
	}
	/**
	* \brief clear
	*/
	void clear() {
	}
	/**
	* \brief set valid
	*/
	void appendValid() {
		appendState(ATTR_BIM_PLANE::VALID);
	}
	/**
	* \brief remove valid
	*/
	void removeValid() {
		removeState(ATTR_BIM_PLANE::VALID);
	}
	/**
	* \brief check attr valid
	*/
	bool isValid() {
		return opAND(ATTR_BIM_PLANE::VALID);
	}
	/**
	* \brief set horiz
	*/
	void appendHoriz() {
		appendState(ATTR_BIM_PLANE::HORIZ);
	}
	/**
	* \brief remove horiz
	*/
	void removeHoriz() {
		removeState(ATTR_BIM_PLANE::HORIZ);
	}
	/**
	* \brief check attr horiz
	*/
	bool isHoriz() {
		return opAND(ATTR_BIM_PLANE::HORIZ);
	}
	/**
	* \brief set vert
	*/
	void appendVert() {
		appendState(ATTR_BIM_PLANE::VERT);
	}
	/**
	* \brief remove vert
	*/
	void removeVert() {
		removeState(ATTR_BIM_PLANE::VERT);
	}
	/**
	* \brief check attr vert
	*/
	bool isVert() {
		return opAND(ATTR_BIM_PLANE::VERT);
	}
	/**
	* \brief set floor
	*/
	void appendFloor() {
		appendState(ATTR_BIM_PLANE::FLOOR);
	}
	/**
	* \brief remove floor
	*/
	void removeFloor() {
		removeState(ATTR_BIM_PLANE::FLOOR);
	}
	/**
	* \brief check attr floor
	*/
	bool isFloor() {
		return opAND(ATTR_BIM_PLANE::FLOOR);
	}
	/**
	* \brief set ceiling
	*/
	void appendCeiling() {
		appendState(ATTR_BIM_PLANE::CEILING);
	}
	/**
	* \brief remove ceiling
	*/
	void removeCeiling() {
		removeState(ATTR_BIM_PLANE::CEILING);
	}
	/**
	* \brief check attr ceiling
	*/
	bool isCeiling() {
		return opAND(ATTR_BIM_PLANE::CEILING);
	}
	/**
	* \brief set wall
	*/
	void appendWall() {
		appendState(ATTR_BIM_PLANE::WALL);
	}
	/**
	* \brief remove wall
	*/
	void removeWall() {
		removeState(ATTR_BIM_PLANE::WALL);
	}
	/**
	* \brief check attr wall
	*/
	bool isWall() {
		return opAND(ATTR_BIM_PLANE::WALL);
	}
	/**
	* \brief set Beam
	*/
	void appendBeam() {
		appendState(ATTR_BIM_PLANE::BEAM);
	}
	/**
	* \brief remove Beam
	*/
	void removeBeam() {
		removeState(ATTR_BIM_PLANE::BEAM);
	}
	/**
	* \brief check attr Beam
	*/
	bool isBeam() {
		return opAND(ATTR_BIM_PLANE::BEAM);
	}
	void reset() {
		opAND(ATTR_BIM_PLANE::NOTVALID);
	}
private:
};

// example
//State_BIM_Plane *state_bim_plane = new State_BIM_Plane();
//state_bim_plane->appendState(ATTR_BIM_PLANE::VALID | ATTR_BIM_PLANE::FLOOR | ATTR_BIM_PLANE::HORIZ);
//DBG_OUT << state_bim_plane->isValid();
//DBG_OUT << state_bim_plane->isFloor();
//DBG_OUT << state_bim_plane->isHoriz();
//state_bim_plane->removeHoriz();
//DBG_OUT << state_bim_plane->isHoriz();

#endif // !_STATE_BIM_H_
