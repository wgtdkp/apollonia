#pragma once

#include "apollonia.h"
#include "base/math.h"
#include "collision.h"
#include "joint.h"

#include <vector>
#include <map>

namespace apollonia {

class Body;

class World {
 public:
  using BodyList = std::vector<Body*>;
  using JointList = std::vector<Joint*>;
  using ArbiterList = std::map<ArbiterKey, Arbiter*>;
  
  World(const Vec2& gravity) : gravity_(gravity) {}
  ~World();
  static Body* NewBody(Float mass, const Vec2& position,
                       const std::vector<Vec2>& vertices);
  static Body* NewBody(Float mass, const Vec2& position,
                       Float width, Float height);
  static Arbiter* NewArbiter(Body& a, Body& b, const Vec2& normal,
      const Arbiter::ContactList& contacts=Arbiter::ContactList());
  static RevoluteJoint* NewRevoluteJoint(Body& a, Body& b, const Vec2& anchor);
  
  void Add(Body* body) { bodies_.push_back(body); }
  void Add(Joint* joint) { joints_.push_back(joint); }
  const Vec2& gravity() const { return gravity_; }
  const BodyList& bodies() const { return bodies_; }
  const JointList& joints() const { return joints_; }

  void Step(Float dt);
  void Clear();

 private:
  void BroadPhase();
  DISABLE_COPY_AND_ASSIGN(World)
  
  Vec2 gravity_ {0, 0};
  size_t iterations_ {10};
  BodyList bodies_;
  JointList joints_;
  ArbiterList arbiters_;
};

}
