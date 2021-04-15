#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with planes.
    // vector from the point on the plane to the pointmass on the cloth at current time
    Vector3D vector_curr = pm.position - point;
    // V * N > 0 then before crossing the plane 
    // V * N < 0 then after crossing the plane 
    // V * N = 0 then at the plane 
    double curr_eval = dot(vector_curr, normal);

    Vector3D vector_before = pm.last_position - point;
    double before_eval = dot(vector_before, normal);

    if ((curr_eval <= 0 && before_eval >= 0) || (curr_eval >= 0 && before_eval <= 0)) {
        //point on the plane closest to pm.position
        Vector3D tangent_point = pm.position - dot(normal.unit(), vector_curr) * normal.unit();
        Vector3D correction = tangent_point - pm.last_position;

        if (dot(vector_before, normal) >= 0) {
            correction = correction + normal * SURFACE_OFFSET ;
        }
        else {
            correction = correction - normal * SURFACE_OFFSET ;
        }
        pm.position = pm.last_position + (1 - friction) * correction;
    }
}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  positions.col(0) << sPoint + 2 * (sCross + sParallel);
  positions.col(1) << sPoint + 2 * (sCross - sParallel);
  positions.col(2) << sPoint + 2 * (-sCross + sParallel);
  positions.col(3) << sPoint + 2 * (-sCross - sParallel);

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
