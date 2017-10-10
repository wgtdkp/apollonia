#include "world.h"
#include "body.h"
#include "collision.h"
#include "joint.h"
#include <algorithm>

namespace apollonia {

World::~World() {
  Clear();
}

PolygonBody* World::NewBox(Float mass,
    Float width, Float height, const Vec2& position) {
  PolygonBody::VertexList vertices = {
    {width/2, height/2}, {-width/2, height/2},
    {-width/2, -height/2}, {width/2, -height/2}
  };
  auto body = new PolygonBody(mass, vertices);
  body->set_position(position);
  return body;
}

PolygonBody* World::NewPolygonBody(Float mass,
    const PolygonBody::VertexList& vertices, const Vec2& position) {
  auto body = new PolygonBody(mass, vertices);
  body->set_position(position);
  return body;
}

Arbiter* World::NewArbiter(Body& a, Body& b, const Vec2& normal,
                           const Arbiter::ContactList& contacts) {
  return new Arbiter(a, b, normal, contacts);
}

RevoluteJoint* World::NewRevoluteJoint(Body& a, Body& b, const Vec2& anchor) {
  return new RevoluteJoint(a, b, anchor);
}

void World::Step(Float dt) {
  // Collide
  for (size_t i = 0; i < bodies_.size(); ++i) {
    for (size_t j = i + 1; j < bodies_.size(); ++j) {
      // TODO(wgtdkp):
      auto& a = dynamic_cast<PolygonBody&>(*bodies_[i]);
      auto& b = dynamic_cast<PolygonBody&>(*bodies_[j]);
      if (!a.ShouldCollide(b)) {
        continue;
      }
      auto arbiter = Collide(&a, &b, dt);
      auto iter = arbiters_.find({a, b});
      if (arbiter == nullptr) {
        if (iter != arbiters_.end()) {
          delete iter->second;
          arbiters_.erase(iter);
        }
      } else if (iter == arbiters_.end()) {
        arbiters_[*arbiter] = arbiter;
      } else {
        auto& old_arbiter = iter->second;
        arbiter->AccumulateImpulse(*old_arbiter);
        delete old_arbiter;
        old_arbiter = arbiter;
      }
    }
  }

  for (auto& kv : arbiters_) {
    kv.second->PreStep(dt);
  }
  for (auto joint : joints_) {
    joint->PreStep(dt);
  }

  // Apply impulse
  for (size_t i = 0; i < iterations_; ++i) {
    for (auto& kv : arbiters_) {
      kv.second->ApplyImpulse();
    }
    for (auto joint : joints_) {
      joint->ApplyImpulse();
    }
  }

  // Integration
  for (auto body : bodies_) {
    body->Integrate(gravity_, dt);
  }
}

void World::Clear() {
  for (auto& kv : arbiters_) {
    delete kv.second;
  }
  arbiters_.clear();
  for (auto joint : joints_) {
    delete joint;
  }
  joints_.clear();
  for (auto body : bodies_) {
    delete body;
  }
  bodies_.clear();
}

};
