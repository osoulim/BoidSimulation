#pragma once

#include <glm/glm.hpp>

#include <vector>

namespace simulation {

using vec2f = glm::vec2;
using vec3f = glm::vec3;

struct Model {
  virtual ~Model() = default;
  virtual void reset() = 0;
  virtual void step(float dt) = 0;
};

struct Particle {
  explicit Particle(vec3f position) : x(position) {}
  Particle(vec3f position, vec3f velocity) : x(position), v(velocity) {}

  vec3f x;
  vec3f v = vec3f{0.f};
};

//
// Particle Model
//
class ParticleModel : public Model {
public:
  ParticleModel();
  void reset() override;
  void step(float dt) override;

public:
  std::vector<Particle> particles;
  float bounds = 10.f;
};

} // namespace simulation
