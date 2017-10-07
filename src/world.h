#pragma once

#include "collision.h"
#include "base/math.h"
#include <vector>
#include <map>

namespace apollonia {

struct Body;
struct Joint;

class World {
 public:
  World(const Vec2& gravity) : gravity_(gravity) {}
  ~World();
  static Body* NewBody(Float mass, const Vec2& position,
                       const std::vector<Vec2>& vertices);
  static Body* NewBody(Float mass, const Vec2& position,
                       Float width, Float height);
  static Arbiter* NewArbiter(Body* a, Body* b, size_t idx,
      const Arbiter::ContactList& contacts=Arbiter::ContactList());
  Joint* NewJoint();
  void Add(Body* body) {
    bodies_.push_back(body);
  }
  void Add(Joint* joint) {
    joints_.push_back(joint);
  }
  const Vec2& gravity() const { return gravity_; }
  const std::vector<Body*>& bodies() const { return bodies_; }
  const std::vector<Joint*>& joints() const { return joints_; }

  void Step(Float dt);
  void Clear();
 private:
  void BroadPhase();

 private:
  Vec2 gravity_ {0, 0};
  size_t iterations_ {10};
  std::vector<Body*> bodies_;
  std::vector<Joint*> joints_;
  std::map<ArbiterKey, Arbiter*> arbiters_;
};

}
