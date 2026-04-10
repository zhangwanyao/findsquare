#ifndef _BIM_FILTER_PLANE_H_
#define _BIM_FILTER_PLANE_H_
#include "../doc/doc_bim.h"
/**
* \brief class of BIM plane filter
*/
class BIM_Filter_Plane {
public:
	/**
	* \brief constructor
	*/
	BIM_Filter_Plane();
	/**
	* \brief destructor
	*/
	~BIM_Filter_Plane();

	/**
	* \brief filter valid plane
	*/
	static void filter_valid(Doc_BIM &doc_bim);
	/**
	* \brief filter horizontal plane
	*/
	static void filter_horiz(Doc_BIM &doc_bim);
	/**
	* \brief filter vertical plane
	*/
	static void filter_vert(Doc_BIM &doc_bim);
	/**
	* \brief filter ceiling
	*/
	static void filter_ceiling(Doc_BIM &doc_bim);
	/**
	* \brief filter ground
	*/
	static void filter_floor(Doc_BIM &doc_bim);
	/**
	* \brief filter wall
	*
	* find plane intxn with union of ceiling and union of floor at same time
	* candidate planes: vertical, not ceiling or floor
	*/
	static void filter_wall(Doc_BIM &doc_bim);
	/**
	* \brief filter beam
	* \added by yu.liang@unre.com
	* \2020/12/15
	* find plane intxn with union of ceiling and vertical planes
	* candidate planes: vertical, not wall
	*/
	static void filter_beam(Doc_BIM &doc_bim);
};

#endif // !_BIM_FILTER_PLANE_H_
