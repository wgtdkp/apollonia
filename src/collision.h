#pragma once

#include "base/math.h"

namespace apollonia {

struct Body;
class World;
class ArbiterKey;

struct Contact {
  Vec2 position;
  Vec2 normal;
  Vec2 ra, rb;
  size_t ia, ib;
  Float separation;
  Float pn {0};
  Float pt {0};
  Float pnb {0};
  Float mass_normal;
  Float mass_tangent;
  Float bias;

  bool operator==(const Contact& other) const {
    return ia == other.ia && ib == other.ib;
  }
  bool operator!=(const Contact& other) const {
    return !(*this == other);
  }
};

class Arbiter {
 public:
  friend class World;
  friend class ArbiterKey;
  static const size_t kNumContacts = 2;
  using ContactList = std::array<Contact, kNumContacts>;
  
  bool operator==(const Arbiter& other) const;
  void ApplyImpulse();
  void UpdateContacts(const Arbiter& arbiter);
  void SetContacts(const Arbiter& arbiter);

 private:
  Arbiter(Body* a, Body* b, const ContactList& contacts)
      : a_(a), b_(b), contacts_(contacts) {}
  Body* a_;
  Body* b_;
  ContactList contacts_;
};

class ArbiterKey {
 public:
  ArbiterKey(const Body* a, const Body* b) : a_(a), b_(b) {}
  ArbiterKey(const Arbiter& arbiter) : ArbiterKey(arbiter.a_, arbiter.b_) {}
  bool operator<(const ArbiterKey& other) const;
  bool operator==(const ArbiterKey& other) const {
    return !(*this < other) && !(other < *this);
  }
  bool operator!=(const ArbiterKey& other) const {
    return !(*this == other);
  }

 private:
  const Body* a_;
  const Body* b_;
};


Arbiter* Collide(Body& a, Body& b);

}
