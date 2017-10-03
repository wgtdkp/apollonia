#include "collision.h"
#include "body.h"

namespace apollonia {

static bool SegmentIntersect(const Vec2& a, const Vec2& b,
                             const Vec2& c, const Vec2& d) {
  auto ac = c - a;
  auto ad = d - a;
  auto bc = c - b;
  auto bd = d - b;
  return Cross(ac, ad) * Cross(bc, bd) <= 0 &&
         Cross(ac, bc) * Cross(ad, bd) <= 0;
}

static std::array<Contact, 2> BodyContact(const Body& a, const Body& b) {
  std::array<Contact, 2> contacts;
  size_t k = 0, na = a.Count(), nb = b.Count();
  for (size_t i = 0; i < na; ++i) {
    for (size_t j = 0; j < nb; ++j) {
      
      if (SegmentIntersect(a.LocalToWorld(a[i]), a.LocalToWorld(a[(i+1)%na]), 
                           b.LocalToWorld(b[i]), b.LocalToWorld(b[(i+1)%nb]))) {
        contacts[k].ia = i;
        contacts[k].ib = j;
        ++k;
      }
    }
  }
  assert(k == 2);
  return contacts;
}

void Collide(Body& a, Body& b) {
  size_t ia, ib;
  Float sa, sb;
  if ((sa = a.FindMinSeparatingAxis(ia, b)) >= 0) {
    return;
  }
  if ((sb = b.FindMinSeparatingAxis(ib, a)) >= 0) {
    return;
  }
  if (sa < sb) {
    std::swap(sa, sb);
    std::swap(ia, ib);
    std::swap(a, b);
  }
  auto normal = a.EdgeAt(ia).Normal();

}

void Arbiter::ApplyImpulse() {

}

}
