#pragma once

#include "apollonia.h"
#include "base/math.h"
#include <vector>

namespace apollonia {

class World;
struct Contact;

/*
enum class BodyType : char {
  kStatic,
  kDynamic
};
*/

class Body {
 public:
  friend class World;
  using VertexList = std::vector<Vec2>;

  bool ShouldCollide(const Body& other) const;
  void ApplyImpulse(const Vec2& impulse, const Vec2& r);
  void Integrate(const Vec2& gravity, Float dt);

  // Convert local point to world
  Vec2 LocalToWorld(const Vec2& local_point) const {
    return position_ + local_point;
  }

  Float mass() const { return mass_; }
  Float inv_mass() const { return inv_mass_; }
  void set_mass(Float mass);

  Float inertia() const { return inertia_; }
  Float inv_inertia() const { return inv_inertia_; }
  void set_inertia(Float inertia);

  const Vec2& centroid() const { return centroid_; }

  const Vec2& position() const { return position_; }
  void set_position(const Vec2& position) { position_ = position; }

  const Mat22& rotation() const { return rotation_; }
  void set_rotation(const Mat22& rotation) { rotation_ = rotation; }
  void set_rotation(Float angle) { set_rotation(Mat22(angle)); }

  const Vec2& velocity() const { return velocity_; }
  void set_velocity(const Vec2& velocity) {velocity_ = velocity; }

  Float angular_velocity() const { return angular_velocity_; }
  void set_angular_velocity(Float angular_velocity) { angular_velocity_ = angular_velocity; }

  const Vec2& force() const { return force_; }
  void set_force(const Vec2& force) { force_ = force; }

  Float torque() const { return torque_; }
  void set_torque(Float torque) { torque_ = torque; }

  Float friction() const { return friction_; }
  void set_friction(Float friction) { friction_ = friction; }

  Float bouncy() const { return bouncy_; }
  void set_bouncy(Float bouncy) { bouncy_ = bouncy; }

 protected:
  Body(Float mass) { set_mass(mass); }
  virtual ~Body() {}
  DISABLE_COPY_AND_ASSIGN(Body)
  
  // Centroid is determined by shape of the body.
  void set_centroid(const Vec2& centroid) { centroid_ = centroid; }

 private:
  Float mass_;
  Float inv_mass_;
  Float inertia_;
  Float inv_inertia_;
  Vec2  centroid_         {0, 0};
  Vec2  position_         {0, 0};
  Mat22 rotation_         {Mat22::I};
  Vec2  velocity_         {0, 0};
  Float angular_velocity_ {0};
  Vec2  force_            {0, 0};
  Float torque_           {0};
  Float friction_         {1};
  Float bouncy_           {0};
};

class PolygonBody : public Body {
 public:
  friend class World;
  using VertexList = std::vector<Vec2>;

  size_t Count() const { return vertices_.size(); }
  
  // Get local vertices with rotation
  Vec2 operator[](size_t idx) const {
    return rotation() * (vertices_[idx] - centroid()) + centroid();
  }
  Vec2 EdgeAt(size_t idx) const {
    return (*this)[(idx+1)%Count()] - (*this)[idx];
  }

  Float FindMinSeparatingAxis(size_t& idx, const PolygonBody& other) const;

 private:
  PolygonBody(Float mass, const VertexList& vertices);
  DISABLE_COPY_AND_ASSIGN(PolygonBody)

  VertexList vertices_;
};

class CircleBody : public Body {
 public:
  friend class World;

 private:
  CircleBody(Float mass, Float radius);
  DISABLE_COPY_AND_ASSIGN(CircleBody)

  Float radius_;
};

}
