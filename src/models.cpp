#include "models.h"

namespace simulation {

//
// Particle Model
//
ParticleModel::ParticleModel() { reset(); }

void ParticleModel::reset() {
  particles.clear();
  // setup
  for (int i = -5; i <= 5; ++i) {
    particles.push_back(Particle({i, i, 0.f}, {-i, 3.f * i, 0.f}));
  }
}

void ParticleModel::step(float dt) {
  for (int iter = 0; iter < 16; ++iter) {
    // do collisions
    for (auto &p : particles) {
      if (length(p.x) > bounds) {
        auto n = normalize(p.x);
        p.v = glm::reflect(p.v, n);
      }
    }

    // move particles
    for (auto &p : particles) {
      // forward Euler
      p.x += p.v * dt;
    }
  }
}

} // namespace simulation
