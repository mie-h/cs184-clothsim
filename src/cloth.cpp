#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.s

    //point masses
    for (int i = 0; i < num_height_points; i++) {
        for (int j = 0; j < num_width_points; j++) {
            double x = (width / num_width_points) * j;
            double y = (height / num_height_points) * i;
            
            bool pin = false;
            if (orientation == HORIZONTAL) {
                for (int k = 0; k < pinned.size(); k++) {
                    if (j == pinned[k][0] && i == pinned[k][1]) {
                        pin = true;
                        break;
                    }
                }
                PointMass p = PointMass(Vector3D(x, 1, y), pin);
                point_masses.push_back(p);
            }
            else if (orientation == VERTICAL) {
                for (int k = 0; k < pinned.size(); k++) {
                    if (j == pinned[k][0] && i == pinned[k][1]) {
                        pin = true;
                        break;
                    }
                }
                double z = (double)rand() / RAND_MAX * 1 / 2000 - 1 / 1000;
                PointMass p = PointMass(Vector3D(x, y, z), pin);
                point_masses.push_back(p);
            }
        }
    }

    //springs
    for (int i = 0; i < num_height_points; i++) {
        for (int j = 0; j < num_width_points; j++) {
            int position = num_width_points * i + j;
            PointMass *pm_center = &point_masses[position];

            //1
            if (i >= 1) {
                int above_index = (i - 1) * num_width_points + j;
                PointMass *pm_above = &point_masses[above_index];

                Spring spring = Spring(pm_center, pm_above, STRUCTURAL);
                springs.push_back(spring);
            }
            if (j >= 1) {
                int left_index = i * num_width_points + (j - 1);
                PointMass* pm_left = &point_masses[left_index];

                Spring spring = Spring(pm_center, pm_left, STRUCTURAL);
                springs.push_back(spring);
            }
            
            //2
            if (i >= 1 && j >= 1) {
                int upper_left_index = (i - 1) * num_width_points + (j - 1);
                PointMass* pm_upper_left = &point_masses[upper_left_index];

                Spring spring = Spring(pm_center, pm_upper_left, SHEARING);
                springs.push_back(spring);
            }
            if (i >= 1 && j < num_width_points - 1) {
                int upper_right_index = (i - 1) * num_width_points + (j + 1);
                PointMass* pm_upper_right = &point_masses[upper_right_index];

                Spring spring = Spring(pm_center, pm_upper_right, SHEARING);
                springs.push_back(spring);
            }

            //3
            if (j >= 2) {
                int left_two_away_index = i * num_width_points + (j - 2);
                PointMass* pm_left_two_away = &point_masses[left_two_away_index];

                Spring spring = Spring(pm_center, pm_left_two_away, BENDING);
                springs.push_back(spring);
            }
            if (i >= 2) {
                int top_two_away_index = (i - 2) * num_width_points + j;
                PointMass* pm_top_two_away = &point_masses[top_two_away_index];

                Spring spring = Spring(pm_center, pm_top_two_away, BENDING);
                springs.push_back(spring);
            }
        }
    }

}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.

  //reset all the forces
  /*for (int i = 0; i < num_height_points; i++) {
      for (int j = 0; j < num_width_points; j++) {
          int position = num_width_points * i + j;
          PointMass* pm_center = &point_masses[position];
          pm_center->forces = Vector3D(0, 0, 0);
      }
  }*/

  //or you can do point_masses.size()
  Vector3D external_force = Vector3D(0, 0, 0);
  for (int i = 0; i < external_accelerations.size(); i++) {
      external_force += external_accelerations[i] * mass;
  }

  int num_masses = point_masses.size();
  for (int i = 0; i < num_masses; i++) {
        //int position = num_width_points * i + j;
        PointMass* pm_center = &point_masses[i];
        pm_center->forces = external_force;
  }

  for (int i = 0; i < springs.size(); i++) {
      Spring spring = springs[i];

      Vector3D dist_vector = spring.pm_a->position - spring.pm_b->position;
      double distance = dist_vector.norm();
      dist_vector.normalize();

      if (cp->enable_structural_constraints && spring.spring_type == STRUCTURAL) {
          
          Vector3D force_spring = dist_vector * cp->ks * (distance - spring.rest_length);
          spring.pm_a->forces -= force_spring;
          spring.pm_b->forces += force_spring;
      }
      if (cp->enable_shearing_constraints && spring.spring_type == SHEARING) {
          Vector3D force_spring = dist_vector * cp->ks * (distance - spring.rest_length);
          spring.pm_a->forces -= force_spring;
          spring.pm_b->forces += force_spring;
      }
      if (cp->enable_bending_constraints && spring.spring_type == BENDING) {
          Vector3D force_spring = dist_vector * cp->ks * 0.2 * (distance - spring.rest_length);
          spring.pm_a->forces -= force_spring;
          spring.pm_b->forces += force_spring;
      }
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (int i = 0; i < num_height_points; i++) {
      for (int j = 0; j < num_width_points; j++) {
          int position = num_width_points * i + j;
          PointMass *pm = &point_masses[position];
          if (pm->pinned == false) {
              Vector3D pos_last = pm->last_position;
              Vector3D pos_curr = pm->position;
              double d = cp->damping / 100.0;
              Vector3D a_t = pm->forces / mass;

              Vector3D pos_next = pos_curr + (1 - d) * (pos_curr - pos_last) + a_t * delta_t * delta_t;
              pm->last_position = pm->position;
              pm->position = pos_next;
          }
      }
  }


  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (int i = 0; i < point_masses.size(); i++) {
      self_collide(point_masses[i], simulation_steps);
  }


  // TODO (Part 3): Handle collisions with other primitives.
  for (int i = 0; i < point_masses.size(); i++) {
      for (CollisionObject* object : *collision_objects) {
          object->collide(point_masses[i]);
      }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (int i = 0; i < springs.size(); i++) {
      Spring spring = springs[i];
      Vector3D dist_vector = spring.pm_a->position - spring.pm_b->position;
      double distance = dist_vector.norm();
      dist_vector.normalize();

      double correction = distance - spring.rest_length - spring.rest_length * 0.1;
      if (correction > 0) {
          if (spring.pm_a->pinned == true && spring.pm_b->pinned == false) {
              spring.pm_b->position += dist_vector * correction;
          }
          if (spring.pm_a->pinned == false && spring.pm_b->pinned == true) {
              spring.pm_a->position -= dist_vector * correction;
          }
          if (spring.pm_a->pinned == false && spring.pm_b->pinned == false) {
              double half_correction = correction / 2;
              spring.pm_a->position -= dist_vector * half_correction;
              spring.pm_b->position += dist_vector * half_correction;
          }
      }
  }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (int i = 0; i < point_masses.size(); i++) {
      PointMass *pm = &point_masses[i];
      float key = hash_position(pm->position);

      if(map.find(key) != map.end()) {
          map[key]->push_back(pm);
      }else{

          map[key] = new vector<PointMass*>;
      }
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

    float key = hash_position(pm.position);
    vector<PointMass*>* boxes = map[key];
    Vector3D correction = Vector3D(0, 0, 0);
    int num_corrections = 0;

    for (PointMass *canditate_pm : *boxes) {
        //not collide with itself
        if (canditate_pm->position == pm.position) {
            continue;
        }
        Vector3D curr_to_candidate = pm.position - canditate_pm->position;
        double distance = curr_to_candidate.norm();
        curr_to_candidate.normalize();
        if (distance < 2 * thickness) {
            num_corrections++;
            correction += (2 * thickness - distance) * curr_to_candidate;
        }
    }

    //average
    if (num_corrections > 0) {
        correction = correction / num_corrections / simulation_steps;
        pm.position += correction;
    }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    double w = 3 * width / num_width_points;
    double h = 3 * height / num_width_points;
    double t = max(w, h);

    double x_sub = pos.x - fmod(pos.x, w);
    double y_sub = pos.y - fmod(pos.y, w);
    double z_sub = pos.z - fmod(pos.z, w);

    int a = x_sub / w;
    int b = y_sub / h;
    int c = z_sub / t;

    return a << 10 ^ b << 5 ^ c;

  //return 0.f; 
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
