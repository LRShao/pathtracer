#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // TODO:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  return false;

}

bool Sphere::intersect(const Ray& r) const {

  // TODO:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

   //Define the variable friom the ray object
   Vector3D oRay = r.o;
   Vector3D d = r.d;

   Vector3D s = oRay - o;

   //Solve for the quadretic equation to find t
   //Assign the coefficients
   double a = d.norm2();
   double b = 2 * dot( s , d );
   double c = s.norm2() - r2 ;

   //*Case 1: discriminant < 0;
   double discriminant = (b * b) - (4 * a * c); 
   if ( discriminant < 0) return false;
   
   //Solve for t1 & t2, t1 < t2
   double t1 = ( -b - sqrt( discriminant ) ) / (2*a) ;
   double t2 = ( -b - sqrt( discriminant ) ) / (2*a) ; 

   //*Case 2: both t1 & t2 is outside the range of t,
   //if not, then the one within the range and samller,
   //should become the intersection parameter t
   double t_min = r.min_t,
          t_max = r.max_t; 

   if ( ( t_min < t1 ) && ( t1 < t_max) )         return true;
   else if ( ( t_min < t2 ) && ( t2 < t_max) )    return true;
   else                                           return false;
}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // TODO:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
   //Define the variable friom the ray object
   Vector3D oRay = r.o;
   Vector3D d = r.d;

   Vector3D s = oRay - o;

   //Solve for the quadretic equation to find t
   //Assign the coefficients
   double a = d.norm2();
   double b = 2 * dot( s , d );
   double c = s.norm2() - r2 ;

   //*Case 1: discriminant < 0;
   double discriminant = (b * b) - (4 * a * c); 
   if ( discriminant < 0) return false;
   
   //Solve for t1 & t2, t1 < t2
   double t1 = ( -b - sqrt( discriminant ) ) / (2*a) ;
   double t2 = ( -b - sqrt( discriminant ) ) / (2*a) ; 

   //*Case 2: both t1 & t2 is outside the range of t,
   //if not, then the one within the range and samller,
   //should become the intersection parameter t
   double t;
   double t_min = r.min_t,
          t_max = r.max_t;   

   if ( ( t_min < t1 ) && ( t1 < t_max) ) t = t1;
   else if ( ( t_min < t2 ) && ( t2 < t_max) ) t = t2;
   else return false;
      
  //Update intersection information
  // t
  i->t = t;

  // n & is_back_hit
  Vector3D n = s + (d * t);
  n = n.unit();

  // is_back_hit
  if ( dot( d , n ) > 0)  
  {
    n = - n;
    i->is_back_hit = true;
  }

  else i->is_back_hit = false;
  
  //n
  i->n = n;  

  // primitive
  i->primitive = this;
  
  // bsdf
  i->bsdf = get_bsdf();

  //update ray.max_t
  r.max_t = t;

  return true;
}

void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CMU462
