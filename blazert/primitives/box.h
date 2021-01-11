//
// Created by tobiask-hfs on 08.01.2021.
//

#ifndef BLAZERT_BOX_H
#define BLAZERT_BOX_H

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <blazert/bvh/options.h>
#include <blazert/datatypes.h>
#include <blazert/ray.h>

#include <blazert/primitives/plane.h>

namespace blazert {

template<typename T>
[[maybe_unused]] constexpr Mat3r<T> xAxisRotation90{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}};
template<typename T>
[[maybe_unused]] constexpr Mat3r<T> yAxisRotation90{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
template<typename T>
[[maybe_unused]] constexpr Mat3r<T> zAxisRotation90{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};

template<typename T>
class Box {

public:
  const Vec3r<T> &center;
  const T dx;
  const T dy;
  const T dz;
  const Mat3r<T> &rotation;
  //const std::array<Plane<T>, 6> BoxPlanes;

  PlaneCollection<T> BoxPlanes;

  unsigned int prim_id;

public:
  Box() = delete;
  Box(const Vec3r<T> &center, const T dx, const T dy, const T dz, const Mat3r<T> &rotation, const unsigned int prim_id)
      : center(center), dx(dx), dy(dy), dz(dz), rotation(rotation), prim_id(prim_id) {

    /** 1. erzeugen der std::vectors wie in plane tests */
    auto centers = std::make_unique<Vec3rList<T>>();
    auto dxx = std::make_unique<std::vector<T>>();
    auto dyy = std::make_unique<std::vector<T>>();
    auto rotations = std::make_unique<Mat3rList<T>>();



    /** 2. Füllen der std::vector */
    const Mat3r<T> rotation_x_plane = xAxisRotation90<T> * rotation;
    const Mat3r<T> rotation_y_plane = yAxisRotation90<T> * rotation;
    const Mat3r<T> rotation_z_plane = rotation;

    // Positive z
    // needs: center(dz), dx, dy, normal rotation
    const Vec3r<T> new_center_pos_z = center + Vec3r<T>{0, 0, dz / static_cast<T>(2.0)};
    centers->emplace_back(new_center_pos_z);
    dxx->emplace_back(dx);
    dyy->emplace_back(dy);
    rotations->emplace_back(rotation_z_plane);

    // Negative z
    // needs: center(dz), dx, dy, normal rotation
    const Vec3r<T> new_center_neg_z = center + Vec3r<T>{0, 0, -dz / static_cast<T>(2.0)};
    centers->emplace_back(new_center_neg_z);
    dxx->emplace_back(dx);
    dyy->emplace_back(dy);
    rotations->emplace_back(rotation_z_plane);

    // Positive x
    // needs: center(dx), dy, dz, rotation from z to +x
    const Vec3r<T> new_center_pos_x = center + Vec3r<T>{dx / static_cast<T>(2.0), 0, 0};
    centers->emplace_back(new_center_pos_x);
    dxx->emplace_back(dy);
    dyy->emplace_back(dz);
    rotations->emplace_back(rotation_x_plane);

    // Negative x
    // needs: center(dx), dy, dz, rotation from z to -x
    const Vec3r<T> new_center_neg_x = center + Vec3r<T>{-dx / static_cast<T>(2.0), 0, 0};
    centers->emplace_back(new_center_pos_x);
    dxx->emplace_back(dy);
    dyy->emplace_back(dz);
    rotations->emplace_back(rotation_x_plane);

    // Positive y
    // needs: center(dy), dx, dz, rotation from z to +y
    const Vec3r<T> new_center_pos_y = center + Vec3r<T>{0, dy / static_cast<T>(2.0), 0};
    centers->emplace_back(new_center_pos_y);
    dxx->emplace_back(dx);
    dyy->emplace_back(dz);
    rotations->emplace_back(rotation_y_plane);

    // Negative y
    // needs: center(dy), dx, dz, rotation form z to -y
    const Vec3r<T> new_center_neg_y = center + Vec3r<T>{0, -dy / static_cast<T>(2.0), 0};
    centers->emplace_back(new_center_neg_y);
    dxx->emplace_back(dx);
    dyy->emplace_back(dz);
    rotations->emplace_back(rotation_y_plane);

    /**
      3. BoxPlanes = PlaneCollection<T>(<vector>);
      PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
      */
    BoxPlanes(*centers, *dxx, *dyy, *rotations);
    /**
     4. intersections
     Iteration über alle Planes die zur Box gehören
        -> prepare_traversal und post_traversal
        -> siehe assert_intersect_primitive_hit in assert_helper.h
      */
  };
  Box(Box &&rhs) noexcept
      : center(std::move(rhs.center)), dx(std::move(rhs.dx)), dy(std::move(rhs.dy)), dz(std::move(rhs.dz)),
        rotation(std::move(rhs.rotation)), prim_id(std::exchange(rhs.prim_id, -1)) {}
  Box &operator=(const Box &rhs) = delete;
};

template<typename T, template<typename A> typename Collection,
         typename = std::enable_if_t<std::is_same<typename Collection<T>::primitive_type, Box<T>>::value>>
[[nodiscard]] inline Box<T> primitive_from_collection(const Collection<T> &collection, const unsigned int prim_idx) {

  const Vec3r<T> &center = collection.centers[prim_idx];
  const T &dx = collection.dxs[prim_idx];
  const T &dy = collection.dys[prim_idx];
  const T &dz = collection.dzs[prim_idx];
  const Mat3r<T> &rotation = collection.rotations[prim_idx];
  return {center, dx, dy, dz, rotation, prim_idx};
};

template<typename T, template<typename A> typename Collection>
class BoxIntersector {
public:
  const Collection<T> &collection;

  Vec3r<T> normal;

  Vec3r<T> ray_org;
  Vec3r<T> ray_dir;
  T min_hit_distance;
  Vec2r<T> uv;
  T hit_distance;
  unsigned int prim_id;

  BoxIntersector() = delete;
  explicit BoxIntersector(const Collection<T> &collection)
      : collection(collection), prim_id(static_cast<unsigned int>(-1)) {}
};

//template<typename T>
//class Box_PlaneCollection {
//public:
//  const Vec3rList<T> &centers;
//  const std::vector<T> &dxs;
//  const std::vector<T> &dys;
//  const std::vector<T> &dzs;
//  const Mat3rList<T> &rotations;
//
//  PlaneCollection<T> plane_pos_x;
//  PlaneCollection<T> plane_neg_x;
//  PlaneCollection<T> plane_pos_y;
//  PlaneCollection<T> plane_neg_y;
//  PlaneCollection<T> plane_pos_z;
//  PlaneCollection<T> plane_neg_z;
//
//  Box_PlaneCollection() = delete;
//  Box_PlaneCollection(const Box_PlaneCollection<T> &rhs) = delete;
//  Box_PlaneCollection(const Vec3rList<T> &centers, const std::vector<T> &dxs, const std::vector<T> &dys,
//                      const std::vector<T> &dzs, const Mat3rList<T> &rotations)
//      : centers(centers), dxs(dxs), dys(dys), dzs(dzs), rotations(rotations) {
//    for (unsigned int prim_id = 0; prim_id < centers.size(); prim_id++) {
//      const Vec3r<T> &center = centers[prim_id];
//      const T dx = dxs[prim_id];
//      const T dy = dys[prim_id];
//      const T dz = dzs[prim_id];
//      const Mat3r<T> &rotation = rotations[prim_id];
//
//      const Vec3r<T> &new_center_pos_z = center + Vec3r<T>{0, 0, dz / static_cast<T>(2.0)};
//      plane_pos_z(new_center_pos_z, dx, dy, rotation);
//      const Vec3r<T> &new_center_neg_z = center + Vec3r<T>{0, 0, -dz / static_cast<T>(2.0)};
//      plane_neg_z(new_center_neg_z, dx, dy, rotation);
//
//    }
//  }
//
//private:
//};
//
//template<typename T>
//class BoxCollection {
//public:
//  typedef BoxIntersector<T, BoxCollection> intersector;
//  typedef Box<T> primitive_type;
//
//  const Vec3rList<T> &centers;
//  const std::vector<T> &dxs;
//  const std::vector<T> &dys;
//  const std::vector<T> &dzs;
//  const Mat3rList<T> &rotations;
//
//  const std::array<Plane<T>, 6> BoxPlanes;
////  Box_PlaneCollection<T> planes;
//  std::vector<std::pair<Vec3r<T>, Vec3r<T>>> box;
//
//  BoxCollection() = delete;
//  BoxCollection(const BoxCollection<T> &rhs) = delete;
//  BoxCollection(const Vec3rList<T> &centers, const std::vector<T> &dxs, const std::vector<T> &dys,
//                const std::vector<T> &dzs, const Mat3rList<T> &rotations)
//      : centers(centers), dxs(dxs), dys(dys), dzs(dzs), rotations(rotations) {
//
//    // pre calculate box
//    box.reserve(centers.size());
//
//    for (unsigned int prim_id = 0; prim_id < centers.size(); prim_id++) {
//      std::cout << "TEST" << std::endl;
//      box.emplace_back(pre_compute_bounding_box(prim_id));
//    }
//  }
//
//  [[nodiscard]] inline unsigned int size() const noexcept { return static_cast<unsigned int>(centers.size()); }
//
//  [[nodiscard]] inline std::pair<Vec3r<T>, Vec3r<T>>
//  get_primitive_bounding_box(const unsigned int prim_id) const noexcept {
//    return box[prim_id];
//  }
//
//  [[nodiscard]] inline Vec3r<T> get_primitive_center(const unsigned int prim_id) const noexcept {
//    return centers[prim_id];
//  }
//
//private:
//  const Vec3r<T> &tmp_center = 0;
//
//  /*  [[nodiscard]] inline std::pair<Vec3r<T>, Vec3r<T>>
//  pre_compute_bounding_box(const unsigned int prim_id) const noexcept {
//    const Vec3r<T> &center = centers[prim_id];
//    const T dx = dxs[prim_id];
//    const T dy = dys[prim_id];
//    const T dz = dzs[prim_id];
//    const Mat3r<T> &rotation = rotations[prim_id];
//
//    // TODO: Calculate bounding box
//    return std::make_pair(std::move(min), std::move(max));
//  }*/
//};
}// namespace blazert

#endif//BLAZERT_BOX_H
