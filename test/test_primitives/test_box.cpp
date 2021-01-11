//
// Created by tobia on 08.01.2021.
//

#define DOCTEST_CONFIG_INCLUDE_TYPE_TRAITS

#include <blazert/bvh/accel.h>
#include <blazert/bvh/builder.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/box.h>
#include <blazert/ray.h>
#include <memory>
//#include <blazert/scene.h>

#include "../test_helpers.h"
#include "assert_helper.h"
#include <third_party/doctest/doctest/doctest.h>

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("Box", T, float, double) {
  const T dx = 2.;
  const T dy = 2.;
  const T dz = 2.;

  auto centers = std::make_unique<Vec3rList<T>>();
  auto dxx = std::make_unique<std::vector<T>>();
  auto dyy = std::make_unique<std::vector<T>>();
  auto dzz = std::make_unique<std::vector<T>>();
  auto rotations = std::make_unique<Mat3rList<T>>();

  dxx->push_back(dx);
  dyy->push_back(dy);
  dzz->push_back(dz);

  SUBCASE("bounding box") {
    SUBCASE("center at origin") {
      const Vec3r<T> center{0., 0., 0.};
      centers->emplace_back(center);

      SUBCASE("non-rotated") {
        const Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);

        BoxCollection<T> boxes(*centers, *dxx, *dyy, *dzz, *rotations);

        const Vec3r<T> true_bmin{-1., -1., -std::numeric_limits<T>::min()};
        const Vec3r<T> true_bmax{1., 1., std::numeric_limits<T>::min()};
        assert_bounding_box(boxes, 0, true_bmin, true_bmax);
      }
    }
  }
}