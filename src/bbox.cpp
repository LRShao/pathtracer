#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CMU462 {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO:
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  //Retrive the ray information
  Vector3D o = r.o;
  Vector3D d = r.d;
  
  //retrive the information of bounding box faces
  double x_min = min.x;
  double y_min = min.y;
  double z_min = min.z;
  double x_max = max.x;
  double y_max = max.y;
  double z_max = max.z;

  double tx_min = (x_min - o.x) / d.x;
  double tx_max = (x_max - o.x) / d.x;
  double ty_min = (y_min - o.y) / d.y;
  double ty_max = (y_max - o.y) / d.y;
  double tz_min = (z_min - o.z) / d.z;
  double tz_max = (z_max - o.z) / d.z;

  if ( tx_min > tx_max ) std::swap( tx_min, tx_max );
  if ( ty_min > ty_max ) std::swap( ty_min, ty_max );
  if ( tz_min > tz_max ) std::swap( tz_min, tz_max );
  
  //First, try to find the smallest possible range of t
  //t_min should be the biggest among the 4 lower bound
  //t_max should be the smallest among the 4 upper bound
  double t_min = std::max( tx_min , std::max( ty_min, std::max( tz_min, r.min_t)));
  double t_max = std::min( tx_max , std::min( ty_max, std::min( tz_max, r.max_t)));
 
  if ( t_min > t_max )  return false;
  else
  { 
     t0 = t_min;
     t1 = t_max;
     return true;
  }
  
}

void BBox::draw(Color c) const {

  glColor4f(c.r, c.g, c.b, c.a);

	// top
	glBegin(GL_LINE_STRIP);
	glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
	glEnd();

	// bottom
	glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
	glEnd();

	// side
	glBegin(GL_LINES);
	glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
	glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
	glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
	glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
	glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CMU462
