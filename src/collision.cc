#include "collision.h"
#include "body.h"
#include "world.h"
#include <algorithm>
#include <GL/glut.h>
#include <GL/glu.h>

namespace apollonia {

using std::abs;

static size_t FindIncidentEdge(const Vec2& normal, const Body& body) {
  size_t idx;  
  auto min_dot = kInf;
  for (size_t i = 0; i < body.Count(); ++i) {
    auto edge_normal = body.EdgeAt(i).Normal();
    auto dot = Dot(edge_normal, normal);
    if (dot < min_dot) {
      min_dot = dot;
      idx = i;
    }
  }
  return idx;
}

static size_t Clip(Arbiter::ContactList& contacts_out,
                   const Arbiter::ContactList& contacts_in,
                   size_t idx, const Vec2& v0, const Vec2& v1) {
  size_t num_out = 0;
  auto normal = (v1 - v0).Normalized();
  auto dist0 = Cross(contacts_in[0].position - v0, normal);
  auto dist1 = Cross(contacts_in[1].position - v0, normal);
  if (dist0 <= 0) {
    contacts_out[num_out++] = contacts_in[0];
  }
  if (dist1 <= 0) {
    contacts_out[num_out++] = contacts_in[1];
  }
  if (dist0 * dist1 < 0) {
    auto total_dist = dist0 - dist1;
    auto v = (contacts_in[0].position * -dist1 + contacts_in[1].position * dist0) / total_dist;
    assert(!std::isnan(v.x) && !std::isnan(v.y));
    contacts_out[num_out].position = v;
    contacts_out[num_out].from_a[0] = true;
    contacts_out[num_out].indices[0] = idx;

    ++num_out;
  }
  assert(num_out <= 2);
  return num_out;
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

  auto va = a.LocalToWorld(a[ia]);
  auto normal = a.EdgeAt(ia).Normal();
  // FIXME(wgtdkp): sign
  auto tangent = normal.Normal();

  auto idx = FindIncidentEdge(normal, b);
  auto next_idx = (idx + 1) % b.Count();
  Arbiter::ContactList contacts = {{b, idx}, {b, next_idx}};
  auto clipped_contacts = contacts;
  for (size_t i = 0; i < a.Count(); ++i) {
    if (i == ia) {
      continue;
    }
    auto v0 = a.LocalToWorld(a[i]);
    auto v1 = a.LocalToWorld(a[(i+1)%a.Count()]);
    auto num = Clip(clipped_contacts, contacts, i, v0, v1);
    if (num < 2) {
      return nullptr;
    }
    assert(num == 2);
    contacts = clipped_contacts;
  }

  auto arbiter = World::NewArbiter(&a, &b, ia);
  for (auto& contact : clipped_contacts) {
    auto sep = Dot(contact.position - va, normal);
    if (sep <= 0) {
      contact.ra = contact.position - a.LocalToWorld(a.center);
      contact.rb = contact.position - b.LocalToWorld(b.center);
      contact.separation = sep;
      contact.mass_normal = 1 / (a.inv_mass + b.inv_mass + Dot(a.inv_inertia * Cross(Cross(contact.ra, normal), contact.ra) + b.inv_inertia * Cross(Cross(contact.rb, normal), contact.rb), normal));
      
      contact.mass_tangent = 1 / (a.inv_mass + b.inv_mass + Dot(a.inv_inertia * Cross(Cross(contact.ra, tangent), contact.ra) + b.inv_inertia * Cross(Cross(contact.rb, tangent), contact.rb), tangent));
            
      contact.bias = -kBiasFactor / dt * std::min(0.0f, contact.separation + kAllowedPenetration);
      arbiter->AddContact(contact);

      glPointSize(4.0f);
      glColor3f(1.0f, 0.0f, 0.0f);
      glBegin(GL_POINTS);
      glVertex2f(contact.position.x, contact.position.y);
      glEnd();
      glPointSize(1.0f);

    }
  }
  return arbiter;
}

Contact::Contact(const Body& b, size_t idx) {
  indices = {{idx, idx}};
  std::fill(from_a.begin(), from_a.end(), false);
  position = b.LocalToWorld(b[idx]);
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
  auto normal = a.EdgeAt(idx_).Normal();
  // FIXME(wgtdkp): sign
  auto tangent = normal.Normal();
  for (auto& contact : contacts_) {
    Vec2 dv = b.velocity + Cross(b.angularVelocity, contact.rb) - a.velocity - Cross(a.angularVelocity, contact.ra);

    auto vn = Dot(dv, normal);
    auto dpn = (-vn + contact.bias) * contact.mass_normal;
    dpn = std::max(contact.pn + dpn, 0.0f) - contact.pn;
    
    Float friction = sqrt(a.friction * b.friction);
    auto vt = Dot(dv, tangent);
    auto dpt = -vt * contact.mass_tangent;
    dpt = std::max(-friction * contact.pn, std::min(friction * contact.pn, contact.pt + dpt)) - contact.pt;
    
    auto p = dpn * normal + dpt * tangent;
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

      auto normal = a_->EdgeAt(idx_).Normal();
      auto tangent = normal.Normal();
      auto p = new_contact.pn * normal + new_contact.pt * tangent;
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
