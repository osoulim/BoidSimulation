#include "models.h"

#include <iostream>

namespace simulation {


	namespace bg = boost::geometry;
	namespace bgi = boost::geometry::index;

	using Point = bg::model::point<float, 3, bg::cs::cartesian>;
	using Box = bg::model::box<Point>;
	using IndexedPoint = std::pair<Point, std::size_t>;


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
		std::vector<IndexedPoint> indexedPoints;

		size_t idGen = 0;
		std::transform(
				particles.begin(), particles.end(),
				back_inserter(indexedPoints),
				[&idGen](Particle const& particle) {
					Point point(particle.position.x, particle.position.y, particle.position.z);
					return std::make_pair(point, idGen++);
				}
		);

		bgi::rtree<IndexedPoint , bgi::quadratic<16> > tree(indexedPoints);

		for (size_t i = 0; i < particles.size(); i++) {
			auto p = particles[i];
			float maxRadius = std::max(separationRadius, std::max(alignmentRadius, cohesionRadius));
			float x = p.position.x, y = p.position.y, z = p.position.z;
			Box queryBox(
		Point(x - maxRadius, y - maxRadius, z - maxRadius),
		Point(x + maxRadius, y - maxRadius, z - maxRadius)
			);

			std::vector<IndexedPoint> neighbors;
			tree.query(bgi::covered_by(queryBox), std::back_inserter(neighbors));

			for (auto &[position, j]: neighbors) {
				if (i == j) {
					continue;
				}
				Particle neighbor = particles[j];
				auto deltaPos = p.position - neighbor.position;
				auto distance = glm::length(deltaPos);
				auto alpha = glm::dot(deltaPos, p.velocity);

				if (distance < separationRadius && alpha > cos(separationRadius)) {
					p.applyForce(calculateSeparationForce(p, neighbor), dt);
				} else if (distance < alignmentRadius && alpha > cos(alignmentAngle)) {
					p.applyForce(calculateAlignmentForce(p, neighbor), dt);
				} else if (distance < cohesionRadius && alpha > cos(cohesionAngle)) {
					p.applyForce(calculateCohesionForce(p, neighbor), dt);
				}
			}

			auto nextPos = p.position + p.velocity * dt;
			if (glm::length(nextPos) > bounds) {
				std::printf("\nvel: %f,%f,%f\n", p.velocity.x, p.velocity.y, p.velocity.z);
				std::printf("pos: %f,%f,%f\n", p.position.x, p.position.y, p.position.z);
				auto n = glm::normalize(p.position);
				p.velocity = glm::reflect(p.velocity, n);
				std::printf("newVel: %f,%f,%f\n", p.velocity.x, p.velocity.y, p.velocity.z);
			}
		}

		// move particles
		for (auto &p : particles) {
			// forward Euler
			p.applyVelocity(dt);
		}
	}

	vec3f ParticleModel::calculateSeparationForce(Particle& a, Particle& b) const {
		auto dist = glm::length(b.position - a.position);
		return -separationConstant * (b.position - a.position) / (dist * dist);
	}

	vec3f ParticleModel::calculateAlignmentForce(Particle &a, Particle &b) const {
		return alignmentConstant * (b.velocity - a.velocity);
	}

	vec3f ParticleModel::calculateCohesionForce(Particle &a, Particle &b) const {
		return cohesionConstant * (b.position - a.position);
	}
} // namespace simulation
