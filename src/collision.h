#pragma once

#include "base/math.h"

namespace apollonia {

struct Body;

struct Contact {
  Vec2 position;
  Vec2 normal;
  Vec2 r1, r2;
  Float separation;
  Float pn {0};
  Float pt {0};
  Float pnb {0};
  Float mass_normal, mass_tangent;
  Float bias;
};

struct Arbiter {
  static const size_t kMaxContacts = 2;
  Body* body_a;
  Body* body_b;
  std::array<Contact, kMaxContacts> contacts;
  size_t num_contact;

  void ApplyImpulse();

};

void Collide(Body* a, Body* b);

}
