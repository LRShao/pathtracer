#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>
#include <cmath>

using std::min;
using std::max;
using std::swap;

namespace CMU462 {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {

    Vector3D z = Vector3D(n.x, n.y, n.z);
    Vector3D h = z;
    if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
    else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
    else h.z = 1.0;

    z.normalize();
    Vector3D y = cross(h, z);
    y.normalize();
    Vector3D x = cross(z, y);
    x.normalize();

    o2w[0] = x;
    o2w[1] = y;
    o2w[2] = z;
}

// Diffuse BSDF //

Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return albedo * (1.0 / PI);
}

Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  //uniform hemisphere sampling
   double random1 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX),
          random2 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);

  //cos and sin work in radians  
  wi->x = sqrt( 1 - (random1 * random1) ) * cos( 2. * PI * random2) ;
  wi->y = sqrt( 1 - (random1 * random1) ) * sin( 2. * PI * random2) ;
  wi->z = random1;
  
  //pdf
  *pdf = 1.0 / ( 2. * PI ); 

  //f is independent of wo or wi
  return albedo * (1.0 / PI);
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  Vector3D wi_reflect;
  reflect(wo, &wi_reflect);
  
  const double error = 1.0e-10;  
 
  if ( (wi - wi_reflect).norm() < error )
  {
     return reflectance;
  }
  else
  {
     Spectrum f0(0.,0.,0.); 
     return f0;
  }
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO:
  // Implement MirrorBSDF
  reflect(wo, wi);  
  *pdf = 1.0;
  return reflectance;
}

// Glossy BSDF //

/*
Spectrum GlossyBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlossyBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0f;
  return reflect(wo, wi, reflectance);
}
*/

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO:
  // Implement RefractionBSDF

  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
 
  Vector3D wi_refract;
  refract(wo, &wi_refract, ior);
 
  Vector3D wi_reflect;
  reflect(wo, &wi_reflect);

  double cos_theta_out = wo[2];
  double cos_theta_in = wi_refract[2];

     //Assign value to necessary variables
     double n_in, n_out;
     if ( cos_theta_out <= 0) 
     {
        n_in = 1.0;
        n_out = ior;
        cos_theta_out = - cos_theta_out;
     }
     else
     {
        n_in = ior;
        n_out = 1.0;
        cos_theta_in = - cos_theta_in;      
     }
     //Calculate the relectance for parallel and perpendicular case
     double r_para = ( ( n_out * cos_theta_in ) - ( n_in * cos_theta_out ) )
                    /( ( n_out * cos_theta_in ) + ( n_in * cos_theta_out ) );   
     double r_perp = ( ( n_in * cos_theta_in ) - ( n_out * cos_theta_out ) )
                    /( ( n_in * cos_theta_in ) + ( n_out * cos_theta_out ) );
     float f_r = ( ( r_para * r_para) + ( r_perp * r_perp ) ) / 2.0;
     float f_t = 1.0 - f_r;   
 
     Spectrum fr (f_r, f_r, f_r);
     Spectrum ft (f_t, f_t, f_t);

     Spectrum f1 ( 1., 1., 1. ); 
     Spectrum f0 ( 0., 0., 0. );   
 
  const double error = 1.0e-10;
  //deternmine the type of incident ray & update wi_refract
  if ( (wi - wi_reflect).norm() < error )
  {
     //This is a reflection case
     //check if it is total internal reflection
     if ( refract(wo, &wi_refract, ior) )     
     {
        //not total internal reflection
        return fr;
     }
     else
     {
        //total internal reflection
         return f1;        
     } 
  }
  else if ( (wi - wi_refract).norm() < error ) return ft;
  else return f0;
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO
  // Compute Fresnel coefficient and reflectance or refractance based on it.
  // deternmine the type of incident ray & update wi

  if ( refract(wo, wi, ior) )
  {
     //not total internal reflection
     //should conform to the Fresnel Equation
     //collect variables

     double cos_theta_out = wo[2];
     double cos_theta_in = (*wi)[2];

     double n_in, n_out;
     if ( cos_theta_out <= 0) 
     {
        n_in = 1.0;
        n_out = ior;
        cos_theta_out = - cos_theta_out;
     }
     else
     {
        n_in = ior;
        n_out = 1.0;
        cos_theta_in = - cos_theta_in;      
     }
     //Calculate the relectance for parallel and perpendicular case
     double r_para = ( ( n_out * cos_theta_in ) - ( n_in * cos_theta_out ) )
                    /( ( n_out * cos_theta_in ) + ( n_in * cos_theta_out ) );   
     double r_perp = ( ( n_in * cos_theta_in ) - ( n_out * cos_theta_out ) )
                    /( ( n_in * cos_theta_in ) + ( n_out * cos_theta_out ) );
     //fraction of reflected light
     float f_r = ( ( r_para * r_para) + ( r_perp * r_perp ) ) / 2.0;
     //fraction of refracted light
     float f_t = 1 - f_r; 

     Spectrum fr (f_r, f_r, f_r);
     Spectrum ft (f_t, f_t, f_t);

     float randomFloat = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    
     if ( randomFloat < f_t ) 
     {
        //currently wi is exactly incident direction in refraction case
        //no need to do anything about it

        *pdf = f_t; 

        return ft;
     }
     else
    {
       Vector3D wi_reflect;
       reflect(wo, &wi_reflect);
       *wi = wi_reflect;

       *pdf = f_r;

       return fr;
    }
  }
  else
  {
     //total internal reflection
     //currently wi is exactly incident direction in the reflection case
     //no need to do anything about it

     *pdf = 1.0;

     Spectrum f1 ( 1., 1., 1. );
     return f1;
  }

  return Spectrum();
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {

  // TODO:
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  Vector3D n( 0., 0., 1. );
//  *wi = n * ( dot( wo , n ) ) * 2.0 - wo ;
  (*wi)[0] = - wo[0];
  (*wi)[1] = - wo[1];
  (*wi)[2] = wo[2];
  
}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {

  // TODO:
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
//  fprintf(stdout, "we are using refract(wo, *wi, ior)\n");

  double cos_theta_out = wo[2];
  double sin_theta_out = sqrt( 1 - cos_theta_out * cos_theta_out );

  //the out going ray is inside the medium
  if ( cos_theta_out < 0)
  {
//     fprintf(stdout, "=====================outgoing ray in medium===========================\n");
//     fprintf(stdout, "wo = (%f, %f, %f)\n", wo.x, wo.y, wo.z);
     //judge if it is total internal reflection
     if ( sin_theta_out > ( 1.0 / ior ) )
     {
//        fprintf(stdout, "sin_theta_out = %f, 1 / ior = %f\n", sin_theta_out, (1.0 / ior) );
//        fprintf(stdout, "Total Internal Reflection \n\n");

        //then the incident ray should also be inside the medium
        //can directly use the reflect function
        reflect(wo, wi);

        return false;
     }
     else
     {
        //else, the incident ray should be in vacuum
        fprintf(stdout, "incident ray in vacuum\n");
        double sin_theta_in = sin_theta_out * ior;
        double cos_theta_in = sqrt( 1 - sin_theta_in * sin_theta_in );  
        
        Vector3D n (0., 0., 1.);
        Vector3D p ( wo - n * cos_theta_out );
        p = p.unit();

        *wi = ( n * cos_theta_in ) - ( p * sin_theta_in ); 
        *wi = wi->unit();

  fprintf(stdout, "wo = (%f, %f, %f)\n", wo.x, wo.y, wo.z);
  fprintf(stdout, "wi = (%f, %f, %f)\n", wi->x, wi->y, wi->z);
  fprintf(stdout, "index of refraction = %f\n\n", ior);        
     }
  }
  //the out going ray is outside the medium, including the critical parellel case
  else 
  {
     //the incident ray should be inside the medium
//     fprintf(stdout, "incident ray in medium\n");
     double sin_theta_in = sin_theta_out / ior;
     double cos_theta_in = - sqrt( 1 - sin_theta_in * sin_theta_in );  
        
     Vector3D n (0., 0., 1.);
     Vector3D p ( wo - n * cos_theta_out );
     p = p.unit();

     *wi = ( n * cos_theta_in ) - ( p * sin_theta_in ); 
     *wi = wi->unit();
  }

  return true;

}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *wi  = sampler.get_sample(pdf);
  return Spectrum();
}

} // namespace CMU462
