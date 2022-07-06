#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

#include "preprocess.h"
#include "vehicle_data_parser.h"
#define PI 3.1415926
struct Point2f {
 public:
  Point2f() : x(0), y(0) {}
  float x;
  float y;
};
Point2f aa(Point2f &pt) {
  Point2f pt2;
  float p[12] = {-0.000287316,  1.78456e-05 ,   -0.656343 ,     2.20572,
  0.00070679,  9.43569e-06 ,    -1.06622 ,    -1.06147,
 2.02272e-06, -0.000762204 ,    0.347031,     0.411426};
  float t = -1.0f * (p[8] * pt.x + p[9] * pt.y + p[10]) / p[11];
  pt2.x = (p[0] * pt.x + p[1] * pt.y + p[2] + t * p[3]) / t;
  pt2.y = (p[4] * pt.x + p[5] * pt.y + p[6] + t * p[7]) / t;
  return pt2;
}

// using namespace Eigen;
//  typedef Matrix<float, 4, 4> Matrix4f;

int main() {
  Eigen::Matrix4f matrix_3f;
  matrix_3f << 1310.686211222937, 0.0, 969.4276982093512, 0.0, 0.0,
      1311.525501008002, 492.43294418115147, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 1.0;
  Eigen::Matrix4f matrix_3f1;
  matrix_3f1 << -0.37658146378227886, 0.023404968353463833, -0.926087796655801, 2.2057193721920902, 0.9263797174575757,
      0.012375149981288672, -0.3763874080019791, -1.0614736995486043, 0.0026511431691445134, -0.9996494704434694,
      -0.02634213217694483, 0.41142628486004407, 0.0, 0.0, 0.0, 1.0;
  auto dc = matrix_3f1.inverse();
  auto aaa = matrix_3f * dc;
  auto ccc = aaa.inverse();
  std::cout << "ccc = " << ccc << std::endl;

  // Eigen::Matrix3f matrix_3f2;
  // matrix_3f2 << 971.241451992899, 0.0, 936.0997507171927, 0.0,
  //     976.4555272496325, 561.801458886737, 0.0, 0.0, 1.0;

  // auto inv = matrix_3f2.inverse();
  // std::cout << "matrix_3f2" << matrix_3f2 << std::endl;
  // std::cout << "inv" << inv << std::endl;
  Point2f p1,p2,p3,p4,p5,p6;
  p1.x = 1420.9957275390625;
  p1.y = 473.9046325683594;
  p2.x = 1454.6761474609375;
  p2.y = 473.9046325683594;

  p3.x = 1421.8992919921875;
  p3.y = 455.4711608886719;
  p4.x = 1454.0159912109375;
  p4.y = 455.4711608886719;

  p5.x = 1521.3470458984375;
  p5.y = 454.1317138671875;
  p6.x = 1537.6719970703125;
  p6.y = 454.1317138671875;
  Point2f r1 = aa(p1);
  Point2f r2 = aa(p2);
  Point2f r3 = aa(p3);
  Point2f r4 = aa(p4);
  Point2f r5 = aa(p5);
  Point2f r6 = aa(p6);
  float a = r2.y - r1.y;
  std::cout << "a = " << a << std::endl;
  float d = r1.x - r2.x;
    std::cout << "d = " << d << std::endl;
  float b = std::sqrt(d * d + a*a);
  std::cout << "b = " << b << std::endl;
   std::cout << "r3.x = " << r3.x<< std::endl;
   std::cout << "r4.x = " << r4.x<< std::endl;
  std::cout << "r5.x = " << r5.x<< std::endl;
   std::cout << "r6.x = " << r6.x<< std::endl;
    std::cout << "r3.y = " << r3.y<< std::endl;
   std::cout << "r4.y = " << r4.y<< std::endl;
  std::cout << "r5.y = " << r5.y<< std::endl;
   std::cout << "r6.y = " << r6.y<< std::endl;
  return 0;
}

