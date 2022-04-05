#pragma once

#include <glm/glm.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

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
		explicit Particle(vec3f position) : position(position) {}
		Particle(vec3f position, vec3f velocity) : position(position), velocity(velocity) {}

		void applyForce(vec3f force, float dt) {
			velocity += force * dt;
		}

		void applyVelocity(float dt) {
			position += velocity * dt;
		}

		vec3f position;
		vec3f velocity = vec3f{0.f};
	};

//
// Particle Model
//
	class ParticleModel : public Model {
	public:
		ParticleModel();
		void reset() override;
		void step(float dt) override;

		vec3f calculateSeparationForce(Particle& a, Particle& b) const;
		vec3f calculateAlignmentForce(Particle& a, Particle& b) const;
		vec3f calculateCohesionForce(Particle& a, Particle& b) const;

	public:
		std::vector<Particle> particles;
		size_t boidsNumber = 100;
		float bounds = 20.f,
			separationRadius = 3.f,
			separationAngle = M_PI * 0.45,
			separationConstant = 1.f,

			alignmentRadius = 5.f,
			alignmentAngle = M_PI * 0.2,
			alignmentConstant = 1.f,

			cohesionRadius = 3.f,
			cohesionAngle = M_PI * 0.5,
			cohesionConstant = 1.0f;
	};

} // namespace simulation
