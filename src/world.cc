#include "world.h"
#include "body.h"
#include "collision.h"
#include "joint.h"
#include <algorithm>

namespace apollonia {

World::~World() {
  for (auto body : bodies_) {
    delete body;
  }
  for (auto joint : joints_) {
    delete joint;
  }
}

Body* World::NewBody(Float mass, const Vec2& position,
                     Float width, Float height) {
  return new Body(mass, position, width, height);
}

void World::Step(Float dt) {
  for (auto body : bodies_) {
    if (body->mass == kInf) {
      continue;
    }
    body->velocity += (gravity_ + body->force / body->mass) * dt;
    body->angularVelocity += (body->torque / body->inertia) * dt;
  }

  for (size_t i = 0; i < bodies_.size(); ++i) {
    for (size_t j = i + 1; j < bodies_.size(); ++j) {
      Collide(bodies_[i], bodies_[j]);
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

};
