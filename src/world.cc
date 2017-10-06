#include "world.h"
#include "body.h"
#include "collision.h"
#include "joint.h"
#include <algorithm>

namespace apollonia {

World::~World() {
  Clear();
}

Body* World::NewBody(Float mass, const Vec2& position,
                     Float width, Float height) {
  return new Body(mass, position, width, height);
}

Arbiter* World::NewArbiter(Body* a, Body* b,
                           const Arbiter::ContactList& contacts) {
  return new Arbiter(a, b, contacts);
}

void World::Step(Float dt) {
  for (size_t i = 0; i < bodies_.size(); ++i) {
    for (size_t j = i + 1; j < bodies_.size(); ++j) {
      if (bodies_[i]->mass == kInf && bodies_[j]->mass == kInf) {
        continue;
      }
      auto arbiter = Collide(*bodies_[i], *bodies_[j], dt);
      if (arbiter == nullptr) {
        arbiters_.erase(ArbiterKey(bodies_[i], bodies_[j]));
        continue;
      }
      auto iter = arbiters_.find(*arbiter);
      if (iter == arbiters_.end()) {
        arbiters_[*arbiter] = arbiter;
        continue;
      } else {
        auto& old_arbiter = iter->second;
        arbiter->AccumulateContacts(*old_arbiter);
        delete old_arbiter;
        old_arbiter = arbiter;
      }
    }
  }

  for (auto body : bodies_) {
    if (body->mass == kInf) {
      continue;
    }
    body->velocity += (gravity_ + body->force * body->inv_mass) * dt;
    body->angularVelocity += (body->torque * body->inv_inertia) * dt;
  }

  for (size_t i = 0; i < iterations_; ++i) {
    for (auto& kv : arbiters_) {
      kv.second->ApplyImpulse();
    }
  }

  for (auto body : bodies_) {
    body->position += body->velocity * dt;
    body->rotation = Mat22(body->angularVelocity * dt) * body->rotation;
    body->force = {0, 0};
    body->torque = 0;
  }

  // TODO(wgtdkp): loop joints
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
