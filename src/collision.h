#pragma once

#include "base/math.h"
#include <vector>

namespace apollonia {

class Body;
class PolygonBody;
class World;
class ArbiterKey;

struct Contact {
  Vec2 position;
  Vec2 ra, rb;
  std::array<bool, 2> from_a;
  std::array<size_t, 2> indices;
  Float separation;
  Float pn {0};
  Float pt {0};
  Float bias {0};  
  Float mass_normal;
  Float mass_tangent;

  Contact(const PolygonBody& b, size_t idx);

  bool operator==(const Contact& other) const {
    if (from_a == other.from_a && indices == other.indices) {
      return true;
    }
    decltype(from_a) from_a_swap = {{from_a[1], from_a[0]}};
    decltype(indices) indices_swap = {{indices[1], indices[0]}};
    return from_a_swap == other.from_a && indices_swap == other.indices;
  }
  bool operator!=(const Contact& other) const {
    return !(*this == other);
  }
};

class Arbiter {
 public:
  friend class World;
  friend class ArbiterKey;
  static const size_t kMaxContacts = 2;
  using ContactList = std::vector<Contact>;
  
  bool operator==(const Arbiter& other) const;
  void PreStep(Float dt);
  void ApplyImpulse();
  void AccumulateImpulse(const Arbiter& old_arbiter);
  void AddContact(const Contact& contact) {
    contacts_.push_back(contact);
    assert(contacts_.size() <= kMaxContacts);
  }

 private:
  Arbiter(Body& a, Body& b, const Vec2& normal, const ContactList& contacts)
      : a_(a), b_(b), normal_(normal), contacts_(contacts) {}
  Body& a_;
  Body& b_;
  Vec2 normal_;
  ContactList contacts_;
};

class ArbiterKey {
 public:
  ArbiterKey(const Body& a, const Body& b) : a_(a), b_(b) {}
  ArbiterKey(const Arbiter& arbiter) : ArbiterKey(arbiter.a_, arbiter.b_) {}
  bool operator<(const ArbiterKey& other) const;
  bool operator==(const ArbiterKey& other) const {
    return !(*this < other) && !(other < *this);
  }
  bool operator!=(const ArbiterKey& other) const {
    return !(*this == other);
  }

 private:
  const Body& a_;
  const Body& b_;
};

Arbiter* Collide(PolygonBody* pa, PolygonBody* pb);

}
