#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>
using namespace std;

namespace CMU462 { namespace StaticScene {

void BVHAccel::partition_node( BVHNode* node )
{
  //Retrieve node information
  int start = node->start,
      range = node->range,
      end = start + range;

   //If the size of the node is greater than allowed size, devide it
   if ( !(node->range <= max_leaf_size) )
   {
      //STEP2 Bin the space
      //Compare the scale of cb in all 3 directions
      BBox bb = node->bb;
      double xScale = bb.max.x - bb.min.x;
      double yScale = bb.max.y - bb.min.y;
      double zScale = bb.max.z - bb.min.z;
     
      //0 for x, 1 for y, 2 for z
      int binDirection;
      if ( (xScale >= yScale) && (xScale >= zScale) ) binDirection = 0;
      if ( (yScale >= xScale) && (yScale >= zScale) ) binDirection = 1;
      if ( (zScale >= xScale) && (zScale >= yScale) ) binDirection = 2;

      //Pick the longest direction, devide it into K bins
    
      //STEP3 3 Triangle-to-bin projection 
      //Calculate the bin id for each triangle according to its centroid
      //Calculate coefficients
      const int number_of_bins = 16;
      const double epsilon = 1.0e-10;

      //Compute the bbox for all triangles' bboxes & the bbox for all centroids
      BBox bboxForCentroids;
  
      for (size_t i = start; i < end; ++i) 
      {
        bboxForCentroids.expand(centroids[i]);
      }
      
      double maxCentroid = bboxForCentroids.max[binDirection];
      double minCentroid = bboxForCentroids.min[binDirection];

      double k = number_of_bins * ( 1 - epsilon) / ( maxCentroid - minCentroid );

      for (int i = start; i < end ; i++)
      {
         //float to int always rounds down
         binID[i] = (int) ( k * ( centroids[i][binDirection] - minCentroid ) );    
      }

      //STEP4 Bin updates
      //Count the number of primitives in each bin
      //Bin access ID is from 0 to 15
      int numberOfTrianglesInBin[number_of_bins]={0};
 
      for (int i = start; i < end ; i++)
      {
         numberOfTrianglesInBin[ binID[i] ]++;
      }

      //Calculathe the bbox of all primitives contained in each bin
     
      BBox bboxOfBin[number_of_bins];

      for (int i = start; i < end ; i++)
      {
         bboxOfBin[ binID[i] ].expand( bboxes[i] );
      }

      //STEP5 Plane evaluations
      //The number of planes is always 1 smaller compared to the number of bins, which is 15
      //Plane ids is from 0 to 14, with plane[0] has bin[0] to its left & bin[1] to its right
      const int number_of_planes = number_of_bins - 1;

      int numberOfTrianglesToTheLeft[number_of_planes];
      double surfaceAreaOfBBoxToTheLeft[number_of_planes];
      int numberOfTrianglesToTheRight[number_of_planes];
      double surfaceAreaOfBBoxToTheRight[number_of_planes];

      //Linearly calculate numberOfTrianglesToTheLeft from the left most plane
      int numberOfTrianglesLeft = 0;
      BBox bboxToTheLeft;
      for (int j = 0; j < number_of_planes; j++)
      {
         numberOfTrianglesLeft += numberOfTrianglesInBin[j];
         numberOfTrianglesToTheLeft[j] = numberOfTrianglesLeft;

         bboxToTheLeft.expand( bboxOfBin[j] );
      //ATTENTION! If bbox is empty, SA should be 0
         if (bboxToTheLeft.empty()) 
            surfaceAreaOfBBoxToTheLeft[j] = 0;
         else
            surfaceAreaOfBBoxToTheLeft[j] = bboxToTheLeft.surface_area(); 
     }

      //Linearly calculate numberOfTrianglesToTheRight from the right most plane
      int numberOfTrianglesRight = 0;
      BBox bboxToTheRight;
      for (int j = ( number_of_planes - 1 ); j >= 0; j--)
      {
         numberOfTrianglesRight += numberOfTrianglesInBin[j+1];
         numberOfTrianglesToTheRight[j] = numberOfTrianglesRight;

         bboxToTheRight.expand( bboxOfBin[j+1] );
      //ATTENTION! If bbox is empty, SA should be 0
         if (bboxToTheRight.empty()) 
            surfaceAreaOfBBoxToTheRight[j] = 0;
         else
            surfaceAreaOfBBoxToTheRight[j] = bboxToTheRight.surface_area();
      }    

      //Evaluate SAH for each plane and find the id of plane with smallest cost
      int partitionPlaneID;
      double lowestCost = std::numeric_limits<double>::max();
      for (int j = 0; j < number_of_planes; j++)
      {
          double cost = numberOfTrianglesToTheLeft[j] * surfaceAreaOfBBoxToTheLeft[j]
                      + numberOfTrianglesToTheRight[j] * surfaceAreaOfBBoxToTheRight[j];
//        fprintf(stdout, "cost[%d] = %f, Counter = %d\n", j, cost ,count);
          if ( cost < lowestCost )
          {
              lowestCost = cost;
              partitionPlaneID = j; 
          } 
      }
     
     //STEP 6 In-place ID list partitioning

     int mid = start + numberOfTrianglesToTheLeft[ partitionPlaneID ];

     int Iter1 = start,
         Iter2 = end - 1;

     //partition the plane, exits when Iter1 == Iter2

     while ( Iter1 < Iter2)
     {
         while ( (binID[Iter1] <= partitionPlaneID) ) Iter1++;
         while ( (binID[Iter2] > partitionPlaneID) ) Iter2--;
         if ( Iter1 < Iter2)
         {
            swap( primitives[Iter1] , primitives[Iter2] );
            swap( bboxes[Iter1] , bboxes[Iter2] );
            swap( centroids[Iter1] , centroids[Iter2] );
            swap( binID[Iter1] , binID[Iter2] );
        
            Iter1++;
            Iter2--;
          }
     }

     //During partioning,also track the triangle bounds and centroid bounds for left and right halves 
     BBox bboxForTrianglesToTheLeft;
     BBox bboxForTrianglesToTheRight;


     for (int i = start; i < mid; i++)
     {
         bboxForTrianglesToTheLeft.expand(bboxes[i]);        
     }
     
     int rangeLeft = numberOfTrianglesToTheLeft[ partitionPlaneID ];

     node->l = new BVHNode(bboxForTrianglesToTheLeft, start, rangeLeft);
     partition_node(node->l);


     for (int i = mid; i < end; i++)
     {
         bboxForTrianglesToTheRight.expand(bboxes[i]);
     }
     
     node->r = new BVHNode(bboxForTrianglesToTheRight, mid, numberOfTrianglesToTheRight[ partitionPlaneID ]);
     partition_node(node->r);
  
   }
}

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  this->primitives = _primitives;
  this->max_leaf_size = max_leaf_size;

  // TODO:
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives

  //STEP 1 Initial Setup
  //compute each primitive(which is triangle)'s bonding box(triangle.get_bbox) & centroid (bbox.centroid())
  //Store them in the vector container bboxes and centroids provided by aggregate class
   
  bboxes.resize( primitives.size() );
  centroids.resize( primitives.size() );
  binID.resize( primitives.size() );

  BBox bboxForTriangles;

  for (size_t i = 0; i < primitives.size(); ++i) {
    bboxes[i] = primitives[i]->get_bbox();
    centroids[i] = bboxes[i].centroid();

    bboxForTriangles.expand(bboxes[i]);    
  }

  root = new BVHNode(bboxForTriangles, 0, primitives.size());
  partition_node(root);

}

BVHAccel::~BVHAccel() {

  // TODO:
  // Implement a proper destructor for your BVH accelerator aggregate

  delete_node(root);
  root = NULL;
}


void BVHAccel::delete_node(BVHNode* node)
{
   if (node->isLeaf()) return;
   else
   {
      delete_node (node->l);
      delete_node (node->r);
      delete node;
   } 
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

bool BVHAccel::intersect(const Ray &ray) const {

  // TODO:
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate.

  bool hit = false;

  Intersection iClosest;
  iClosest.t = ray.max_t;
    
  hit = find_closest_hit(ray, root , iClosest);

  return hit;

}

bool BVHAccel::find_closest_hit(const Ray& ray, const BVHNode* node, Intersection& iClosest) const
 {
   bool hit = false;

   //t_min, t_max to store the range of t returned by BBox::intersect
   double t_min, t_max;   

   if ( ( ! node->bb.intersect(ray, t_min , t_max) ))   return false;
   else if ( t_min > iClosest.t )                       return false;  

   if ( node->isLeaf() )
   {
      for ( int i = node->start; i < ( node->start + node->range ); i++)
      {
         Intersection iTemp;
         bool hitTemp; 
         hitTemp = primitives[i]->intersect( ray, &iTemp);
  
         if ( ( hitTemp ) && (iTemp.t < iClosest.t) )
             {           
                //update intersection information
                iClosest = iTemp;
                hit = hitTemp;
             }
      }

      return hit;
   }
   else
   {
      //Front-to-back optimization
      double t_min_l, t_max_l,
             t_min_r, t_max_r,
             t_min_second;

      node->l->bb.intersect(ray, t_min_l , t_max_l);
      node->r->bb.intersect(ray, t_min_r , t_max_r);

      BVHNode* firstNode;
      BVHNode* secondNode;

      if (t_min_l <= t_min_r)
      { 
        firstNode = node->l;
        secondNode = node->r;
        t_min_second = t_min_r;
      }
      else
      {
        firstNode = node->r;
        secondNode = node->l;
        t_min_second = t_min_l;
      }  
         
      hit = find_closest_hit ( ray, firstNode, iClosest);

      if ( t_min_second <= iClosest.t)
      {
         //If ray does not hit any triangle in second node, then there is no need to update hit status
         bool hitTemp; 
         hitTemp = find_closest_hit ( ray, secondNode, iClosest);
         if (hitTemp) hit = hitTemp;
      }
      return hit;
   }
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i) const {

  // TODO:
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate. When an intersection does happen.
  // You should store the non-aggregate primitive in the intersection data
  // and not the BVH aggregate itself.

  bool hit = false;

  Intersection iClosest;
  iClosest.t = ray.max_t;
    
  hit = find_closest_hit(ray, root , iClosest);
 
  // Pass the information of intersection
  if (hit) 
  {
      i->t = iClosest.t;
      i->primitive = iClosest.primitive;
      i->n = iClosest.n;
      i->bsdf = iClosest.bsdf;
      i->is_back_hit = iClosest.is_back_hit;
  }

  return hit;

}

}  // namespace StaticScene
}  // namespace CMU462  
