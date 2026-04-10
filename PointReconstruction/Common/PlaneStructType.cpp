#include "PlaneStructType.h"

StructuredHole & StructuredHole::operator = (const StructuredHole &structHole)
{
	if (this != &structHole)
	{
		this->id = structHole.id;
		this->type = structHole.type;
		this->vertice = structHole.vertice;
		this->vertice_s = structHole.vertice_s;
		this->height = structHole.height;
		this->width = structHole.width;
	}
	return *this;
}


StructuredPlane & StructuredPlane::operator = (const StructuredPlane &structPlane)
{
	if (this != &structPlane)
	{
		this->id = structPlane.id;
		this->normal = structPlane.normal;
		this->center = structPlane.center;
		this->reflectImg = structPlane.reflectImg;
		this->normalImg = structPlane.normalImg;
		this->direction = structPlane.direction;
		this->type = structPlane.type;
		this->vertices = structPlane.vertices;
		this->vertices_s = structPlane.vertices_s;
		this->holes = structPlane.holes;
		this->truePlane = structPlane.truePlane;
		this->wall_height = structPlane.wall_height;
		this->wall_width = structPlane.wall_width;
	}
	return *this;
}


