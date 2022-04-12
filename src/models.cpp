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

	void ParticleModel::reset(int inputBoidsNumber) {
		particles.clear();
		// setup
		for (size_t i = 0; i < inputBoidsNumber; ++i) {
			auto random = [](float low, float high) {
				return low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high - low)));
			};
			particles.push_back(Particle(
					{random(-bounds/2, bounds/2), random(-bounds/2, bounds/2), random(-bounds/2, bounds/2)},
					{random(-1.f, 1.f), random(-1.f, 1.f), random(-1.f, 1.f)}));
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
			auto &p = particles[i];
			float maxRadius = std::max(separationRadius, std::max(alignmentRadius, cohesionRadius));
			float x = p.position.x, y = p.position.y, z = p.position.z;
			Box queryBox(
		Point(x - maxRadius, y - maxRadius, z - maxRadius),
		Point(x + maxRadius, y + maxRadius, z + maxRadius)
			);

			std::vector<IndexedPoint> neighbors;
			tree.query(bgi::covered_by(queryBox), std::back_inserter(neighbors));

//			std::cout<<neighbors.size()<<std::endl;
			for (auto &[position, j]: neighbors) {
				if (i == j) {
					continue;
				}
				Particle neighbor = particles[j];
				auto deltaPos = p.position - neighbor.position;
				auto distance = glm::length(deltaPos);
				auto alpha = glm::dot(deltaPos, p.velocity);

				if (distance < separationRadius && alpha > cos(separationRadius)) {
//					std::cout<<i<<" "<<j<<":separation\n";
					p.applyForce(calculateSeparationForce(p, neighbor), dt);
				}
				if (distance < alignmentRadius && alpha > cos(alignmentAngle)) {
//					std::cout<<i<<" "<<j<<":alignment\n";
					p.applyForce(calculateAlignmentForce(p, neighbor), dt);
				}
				if (distance < cohesionRadius && alpha > cos(cohesionAngle)) {
//					std::cout<<i<<" "<<j<<":cohesion\n";
					p.applyForce(calculateCohesionForce(p, neighbor), dt);
				}
			}

//			if (glm::length(p.position) > bounds) {
//				auto n = glm::normalize(p.position);
//				p.velocity = glm::reflect(p.velocity, n);
//			}

			// cube collision
			for (int axis = 0; axis < 3; axis++) {
				if (p.position[axis] < -bounds) {
					p.velocity[axis] += turnFactor;
				} else if (p.position[axis] > bounds) {
					p.velocity[axis] -= turnFactor;
				}
			}


			p.applyVelocityLimit(velocityLimit);
		}

		// move particles
		for (auto &p : particles) {
			// forward Euler
			p.applyVelocity(dt);
		}
		auto position = particles[0].position;
		auto speed = particles[0].velocity;
		printf("particle0 loc and speed %f,%f,%f - %f,%f,%f \n", position.x, position.y, position.z, speed.x, speed.y, speed.z);
	}

	vec3f ParticleModel::calculateSeparationForce(Particle& a, Particle& b) const {
		auto dist = glm::length(a.position - b.position);
		return separationConstant * (a.position - b.position) / (dist * dist);
	}

	vec3f ParticleModel::calculateAlignmentForce(Particle &a, Particle &b) const {
		return alignmentConstant * (b.velocity - a.velocity);
	}

	vec3f ParticleModel::calculateCohesionForce(Particle &a, Particle &b) const {
		return cohesionConstant * (b.position - a.position);
	}
} // namespace simulation
