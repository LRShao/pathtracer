#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 { namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
    mesh(mesh), v1(v1), v2(v2), v3(v3) { }

BBox Triangle::get_bbox() const {
  
  // TODO: 
  // compute the bounding box of the triangle
     Vector3D p0 = mesh->positions[v1],
              p1 = mesh->positions[v2],
              p2 = mesh->positions[v3]; 

  BBox bb(p0);
  bb.expand(p1);
  bb.expand(p2); 
  
  return bb;
}

bool Triangle::intersect(const Ray& r) const {
  
  // TODO: implement ray-triangle intersection
  
  //Retrieve information
  //triangle vertices 
  Vector3D p0 = mesh->positions[v1],
           p1 = mesh->positions[v2],
           p2 = mesh->positions[v3];

  //ray origin & direction
  Vector3D o = r.o;
  Vector3D d = r.d;
  
  //Define the vectors for computation
  Vector3D e1 = p1 - p0,
           e2 = p2 - p0,
           s  = o  - p0;
   
  //Solve for u,v,t
  //Computer 4 determinats
  //Denominator
  double denominator = dot( cross( e1, e2 ) , -d );

  //*There is two case that will not be considered as hitting
  //*Case 1: ray parallel to the triangle
  if ( denominator == 0) return false;

  //Compute other 3 determinants
  double d1 = dot( cross( s, e2 ) , -d );
  double d2 = dot( cross( e1, s ) , -d );
  double d3 = dot( cross( e1, e2 ) , s );

  //u,v,t
  double u = d1 / denominator;
  double v = d2 / denominator;
  double t = d3 / denominator;

  //*Case 2: u < 0 || v < 0 || u + v > 1, the intersection is outside the triangle || t outside range
  if ( (u  < 0 ) || (v < 0 ) || ( (1 - u - v) < 0 ) || ( t < r.min_t) || ( t > r.max_t) ) return false;

  //update ray.max_t
  r.max_t = t;

  return true;
}

bool Triangle::intersect(const Ray& r, Intersection *isect) const {
  
  // TODO: 
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  
  //Retrieve information
  //triangle vertices 
  Vector3D p0 = mesh->positions[v1],
           p1 = mesh->positions[v2],
           p2 = mesh->positions[v3];

  //ray origin & direction
  Vector3D o = r.o;
  Vector3D d = r.d;
  
  //Define the vectors for computation
  Vector3D e1 = p1 - p0,
           e2 = p2 - p0,
           s  = o  - p0;
   
  //Solve for u,v,t
  //Computer 4 determinats
  //Denominator
  double denominator = dot( cross( e1, e2 ) , -d );

  //*There are two cases that will not be considered as hitting
  //*Case 1: ray parallel to the triangle
  if ( denominator == 0)  return false;

  //Compute other 3 determinants
  double d1 = dot( cross( s, e2 ) , -d );
  double d2 = dot( cross( e1, s ) , -d );
  double d3 = dot( cross( e1, e2 ) , s );

  //u,v,t
  double u = d1 / denominator;
  double v = d2 / denominator;
  double t = d3 / denominator;

  //*Case 2: u < 0 || v < 0 || u + v > 1, the intersection is outside the triangle || t outside range
  if ( (u < 0 ) || (v < 0) || ( (1 - u - v) < 0 ) || ( t < r.min_t) || ( t > r.max_t) ) return false;

  
  //Update intersection information
  // t
  isect->t = t;

  // n & is_back_hit
  //Retrive three vector noramls
  Vector3D vertexNormal0 = mesh->normals[v1] ,
           vertexNormal1 = mesh->normals[v2] ,
           vertexNormal2 = mesh->normals[v3] ;

  //compute weight
  double w0 = 1 - u - v,
         w1 = u,
         w2 = v;
 
  //interpolate point normal using barycentric coordinates
  Vector3D n = (vertexNormal0 * w0) + (vertexNormal1 * w1) + (vertexNormal2 * w2);
  n = n.unit();

  // is_back_hit
  if ( dot( d , n ) > 0)  
  {
    n = - n;
    isect->is_back_hit = true;
  }

  else isect->is_back_hit = false;
  
  //n
  isect->n = n;  

  // primitive
  isect->primitive = this;
  
  // bsdf
  isect->bsdf = get_bsdf();

  //update ray.max_t
  r.max_t = t;

  return true;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}



} // namespace StaticScene
} // namespace CMU462
