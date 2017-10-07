#include "collision.h"
#include "body.h"
#include "world.h"
#include <algorithm>

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
  auto total = (abs(Cross(ac, cd)) + abs(Cross(bc, cd)));
  if (total == 0) {
    if (Dot(ac, bc) < 0) {
      return c;
    }
    return a;
  }
  auto ratio = abs(Cross(ac, cd)) / total;
  return a + ab * ratio;
}

Arbiter* Collide(Body* pa, Body* pb, Float dt) {
  static const Float kAllowedPenetration = 0.01;
  static const Float kBiasFactor = 0.2;
  size_t ia, ib;
  Float sa, sb;
  if ((sa = pa->FindMinSeparatingAxis(ia, *pb)) >= 0) {
    return nullptr;
  }
  if ((sb = pb->FindMinSeparatingAxis(ib, *pa)) >= 0) {
    return nullptr;
  }
  if (sa < sb) {
    std::swap(sa, sb);
    std::swap(ia, ib);
    std::swap(pa, pb);
  }

  auto& a = *pa;
  auto& b = *pb;

  auto normal = a.EdgeAt(ia).Normal();
  auto ab = b.LocalToWorld(b.center) - a.LocalToWorld(a.center);
  normal *= Dot(ab, normal) > 0 ? 1 : -1;

  auto tangent = normal.Normal();
  size_t na = a.Count(), nb = b.Count();
  auto arbiter = World::NewArbiter(&a, &b);
  for (size_t i = 0; i < na; ++i) {
    for (size_t j = 0; j < nb; ++j) {
      auto va1 = a.LocalToWorld(a[i]), va2 = a.LocalToWorld(a[(i+1)%na]);
      auto vb1 = b.LocalToWorld(b[j]), vb2 = b.LocalToWorld(b[(j+1)%nb]);
      if (!SegmentIntersect(va1, va2, vb1, vb2)) {
        continue;
      }
      Contact contact;
      contact.position = SegmentIntersection(va1, va2, vb1, vb2);
      contact.normal = normal;
      contact.ra = contact.position - a.LocalToWorld(a.center);
      contact.rb = contact.position - b.LocalToWorld(b.center);
      contact.ia = i;
      contact.ib = j;
      contact.separation = sa;

      contact.mass_normal = 1 / (a.inv_mass + b.inv_mass + Dot(a.inv_inertia * Cross(Cross(contact.ra, normal), contact.ra) + b.inv_inertia * Cross(Cross(contact.rb, normal), contact.rb), normal));

      contact.mass_tangent = 1 / (a.inv_mass + b.inv_mass + Dot(a.inv_inertia * Cross(Cross(contact.ra, tangent), contact.ra) + b.inv_inertia * Cross(Cross(contact.rb, tangent), contact.rb), tangent));
      
      contact.bias = -kBiasFactor / dt * std::min(0.0f, contact.separation + kAllowedPenetration);
      arbiter->AddContact(contact);
    }
  }
  return arbiter;
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
    auto dpn = (-vn + contact.bias) * contact.mass_normal;
    dpn = std::max(contact.pn + dpn, 0.0f) - contact.pn;
    
    auto tangent = contact.normal.Normal();
    Float friction = sqrt(a.friction * b.friction);
    auto vt = Dot(dv, tangent);
    auto dpt = -vt * contact.mass_tangent;
    dpt = std::max(-friction * contact.pn, std::min(friction * contact.pn, contact.pt + dpt)) - contact.pt;
    
    auto p = dpn * contact.normal + dpt * tangent;
    a.velocity -= p * a.inv_mass;
    a.angularVelocity -= a.inv_inertia * Cross(contact.ra, p);
    b.velocity += p * b.inv_mass;
    b.angularVelocity += b.inv_inertia * Cross(contact.rb, p);
    contact.pn += dpn;
    contact.pt += dpt;
  }
}

void Arbiter::AccumulateImpulse(const Arbiter& old_arbiter) {
  const auto& old_contacts = old_arbiter.contacts_;
  for (auto& new_contact : contacts_) {
    auto old_contact = std::find(old_contacts.begin(), old_contacts.end(), new_contact);
    if (old_contact != old_contacts.end()) {
      new_contact.pn = old_contact->pn;
      new_contact.pt = old_contact->pt;
      new_contact.pnb = old_contact->pnb;

      auto p = new_contact.pn * new_contact.normal + new_contact.pt * new_contact.normal.Normal();
      a_->velocity -= a_->inv_mass * p;
      a_->angularVelocity -= a_->inv_inertia * Cross(new_contact.ra, p);
      b_->velocity += b_->inv_mass * p;
      b_->angularVelocity += b_->inv_inertia * Cross(new_contact.rb, p);
    }
  }
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
