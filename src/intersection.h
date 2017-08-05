#ifndef CMU462_INTERSECT_H
#define CMU462_INTERSECT_H

#include <vector>

#include "CMU462/vector3D.h"
#include "CMU462/spectrum.h"
#include "CMU462/misc.h"

#include "bsdf.h"

namespace CMU462 { namespace StaticScene {

class Primitive;

/**
 * A record of an intersection point which includes the time of intersection
 * and other information needed for shading
 */
struct Intersection {

/**
 * Constructor.
 * Store information about the detail of the hit.
 * \param t the ray's t-value of the hit point
 * \param n the normal of the surface at the hit point (interpolated normal
 *  via barycentric coordinates
 * \param primitive a pointer to the primitive that was hit
 * \param bsdf a pointer to the surface brdf at the hit point
 *  obtained via mesh->get_bsdf(); 
 * \param is_back_hit
 */

//  Intersection(double t, Primitive* primitive, Vector3D n, BSDF* bsdf, bool is_back_hit)
//              : t(t), primitive(primitive), n(n), bsdf(bsdf), is_back_hit(is_back_hit) {}  

  Intersection() : t (INF_D), primitive(NULL), bsdf(NULL) { }

  double t;    ///< time of intersection

  const Primitive* primitive;  ///< the primitive intersected

  Vector3D n;  ///< normal at point of intersection

  BSDF* bsdf; ///< BSDF of the surface at point of intersection

  // More to follow.

  bool is_back_hit; ///< if the ray intersects with the back of the surface (i.e. d * n >0)
};

} // namespace StaticScene
} // namespace CMU462

#endif // CMU462_INTERSECT_H
