#pragma once

#include "base/math.h"

namespace apollonia {

struct Body;

struct Contact {
  Vec2 position;
  Vec2 normal;
  Vec2 ra, rb;
  size_t ia, ib;
  Float separation;
  Float pn {0};
  Float pt {0};
  Float pnb {0};
  Float mass_normal, mass_tangent;
  Float bias;

  Contact(const Body& a, size_t ia, const Body& b, size_t ib);
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
