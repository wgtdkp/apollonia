#include "collision.h"
#include "body.h"
#include "world.h"
#include <algorithm>

#include <iostream>
#include <string>
#include <cstdio>

static size_t cnt = 0;

namespace apollonia {

using std::abs;

static bool SegmentIntersect(const Vec2& a, const Vec2& b,
                             const Vec2& c, const Vec2& d) {
  auto ac = c - a, ad = d - a;
  auto bc = c - b, bd = d - b;
  return (Cross(ac, ad) * Cross(bc, bd) < 0 || Cross(ac, ad) == 0) &&
         (Cross(ac, bc) * Cross(ad, bd) < 0 || Cross(ac, bc) == 0);
}

static Vec2 SegmentIntersection(const Vec2& a, const Vec2& b,
                                const Vec2& c, const Vec2& d) {
  auto ab = b - a, cd = d - c;
  auto ac = c - a, bc = c - b;
  auto ratio = abs(Cross(ac, cd)) / (abs(Cross(ac, cd)) + abs(Cross(bc, cd)));
  return a + ab * ratio;
}

Arbiter* Collide(Body& a, Body& b) {
  size_t ia, ib;
  Float sa, sb;
  if ((sa = a.FindMinSeparatingAxis(ia, b)) >= 0) {
    return nullptr;
  }
  if ((sb = b.FindMinSeparatingAxis(ib, a)) >= 0) {
    return nullptr;
  }
  if (sa < sb) {
    std::swap(sa, sb);
    std::swap(ia, ib);
    std::swap(a, b);
  }
  
  auto normal = a.EdgeAt(ia).Normal();
  auto tangent = normal.Normal();
  Arbiter::ContactList contacts;
  size_t k = 0, na = a.Count(), nb = b.Count();
  for (size_t i = 0; i < na; ++i) {
    for (size_t j = 0; j < nb; ++j) {
      auto va1 = a.LocalToWorld(a[i]), va2 = a.LocalToWorld(a[(i+1)%na]);
      auto vb1 = b.LocalToWorld(b[j]), vb2 = b.LocalToWorld(b[(j+1)%nb]);
      if (!SegmentIntersect(va1, va2, vb1, vb2)) {
        continue;
      }
      assert(k < Arbiter::kNumContacts);
      auto& contact = contacts[k];
      contact.position = SegmentIntersection(va1, va2, vb1, vb2);
      contact.normal = normal;
      contact.ra = contact.position - a.LocalToWorld(a.center);
      contact.rb = contact.position - b.LocalToWorld(b.center);
      contact.ia = i;
      contact.ib = j;
      contact.separation = -abs(Dot(va1 - contact.position, normal));

      contact.mass_normal = 1 / (1 / a.mass + 1 / b.mass + Dot(1 / a.inertia * Cross(Cross(contact.ra, normal), contact.ra) + 1 / b.inertia * Cross(Cross(contact.rb, normal), contact.rb), normal));

      contact.mass_tangent = 1 / (1 / a.mass + 1 / b.mass + Dot(1 / a.inertia * Cross(Cross(contact.ra, tangent), contact.ra) + 1 / b.inertia * Cross(Cross(contact.rb, tangent), contact.rb), tangent));
      
      contact.bias = 0;
      ++k;
    }
  }
  assert(k == 2);
  return World::NewArbiter(&a, &b, contacts);
}

bool Arbiter::operator==(const Arbiter& other) const {
  if (ArbiterKey(*this) != ArbiterKey(other)) {
    return false;
  }
  for (size_t i = 0; i < contacts_.size(); ++i) {
    if (contacts_[i] != other.contacts_[i]) {
      return false;
    }
  }
  return true;
}

void Arbiter::ApplyImpulse() {
  auto& a = *a_;
  auto& b = *b_;
  for (auto& contact : contacts_) {
    Vec2 dv = b.velocity + Cross(b.angularVelocity, contact.rb) - a.velocity - Cross(a.angularVelocity, contact.ra);

    auto vn = Dot(dv, contact.normal);
    auto dpn = -vn * contact.mass_normal;
    dpn = std::max(contact.pn + dpn, 0.0f) - contact.pn;
    auto pn = dpn * contact.normal;
    a.velocity -= pn / a.mass;
    a.angularVelocity -= 1 / a.inertia * Cross(contact.ra, pn);
    b.velocity += pn / b.mass;
    b.angularVelocity += 1 / b.inertia * Cross(contact.rb, pn);
    contact.pn += dpn;

    dv = b.velocity + Cross(b.angularVelocity, contact.rb) - a.velocity - Cross(a.angularVelocity, contact.ra);

    auto tangent = contact.normal.Normal();
    Float friction = sqrt(a.friction * b.friction);
    auto vt = Dot(dv, tangent);
    auto dpt = -vt * contact.mass_tangent;
    dpt = std::max(-friction * contact.pn, std::min(friction * contact.pn, contact.pt + dpt)) - contact.pt;
    auto pt = dpt * tangent;
    a.velocity -= pt / a.mass;
    a.angularVelocity -= 1 / a.inertia * Cross(contact.ra, pt);
    b.velocity += pt / b.mass;
    b.angularVelocity += 1 / b.inertia * Cross(contact.rb, pt);
    contact.pt += dpt;
  }
}

void Arbiter::UpdateContacts(const Arbiter& arbiter) {
  for (size_t i = 0; i < contacts_.size(); ++i) {
    auto pn = contacts_[i].pn;
    auto pt = contacts_[i].pt;
    auto pnb = contacts_[i].pnb;
    contacts_[i] = arbiter.contacts_[i];
    contacts_[i].pn = pn;
    contacts_[i].pt = pt;
    contacts_[i].pnb = pnb;
  }
}

void Arbiter::SetContacts(const Arbiter& arbiter) {
  assert(ArbiterKey(*this) == ArbiterKey(arbiter));
  contacts_ = arbiter.contacts_;
}

bool ArbiterKey::operator<(const ArbiterKey& other) const {
  auto a1 = this->a_, b1 = this->b_;
  auto a2 = other.a_, b2 = other.b_;
  if (a1 > b1) {
    std::swap(a1, b1);
  }
  if (a2 > b2) {
    std::swap(a2, b2);
  }
  if (a1 != a2) {
    return a1 < a2;
  }
  return b1 < b2;
}

}
