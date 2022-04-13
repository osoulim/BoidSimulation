#pragma once

#include <glm/glm.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <vector>

namespace simulation {

	namespace bg = boost::geometry;
	namespace bgi = boost::geometry::index;

	using Point = bg::model::point<float, 3, bg::cs::cartesian>;
	using Box = bg::model::box<Point>;
	using IndexedPoint = std::pair<Point, std::size_t>;

	using vec2f = glm::vec2;
	using vec3f = glm::vec3;

	using vec3i = glm::ivec3;

	struct Particle {
		explicit Particle(vec3f position) : position(position) {}
		Particle(vec3f position, vec3f velocity, bool isBarrier = false)
				: position(position), velocity(velocity), isBarrier(isBarrier) {}

		void applyForce(vec3f force, float dt) {
			if (isBarrier) {
				return;
			}
			velocity += force * dt;
		}

		void applyVelocity(float dt) {
			if (isBarrier) {
				return;
			}
			position += velocity * dt;
		}

		void applyVelocityLimit(float limit) {
			if (isBarrier) {
				return;
			}
			auto velocityAmount = glm::length(velocity);
			if (velocityAmount > limit) {
				velocity = velocity / velocityAmount * limit;
			}
		}

		vec3f position;
		vec3f velocity = vec3f{0.f};

		bool isBarrier = false;
	};

	struct SpatialStructure {
		SpatialStructure() = default;
		virtual ~SpatialStructure() = default;
		virtual std::vector<int> getNeighbours(vec3f, float) {return {};};
	};

	class BoostRTree: public SpatialStructure {
	public:
		BoostRTree() = default;

		std::vector<int> getNeighbours(vec3f, float) override;
		explicit BoostRTree(std::vector<Particle> const &);

	private:
		std::unique_ptr<bgi::rtree<IndexedPoint , bgi::quadratic<16>>> tree;
	};

	class MamziIndex: public SpatialStructure {
	public:
		MamziIndex() = default;
		std::vector<int> getNeighbours(vec3f, float) override;
		explicit MamziIndex(std::vector<Particle> const &, float);

	private:
		std::map<std::vector<int>, std::vector<int>> spatialMap;
		static std::vector<int> getIndex(vec3f, float);
	};

	struct Model {
		std::vector<Particle> particles;
		virtual ~Model() = default;
		virtual void reset(int) = 0;
		virtual void step(float dt, int indexingMethod) {
			float maxRadius = std::max(separationRadius, std::max(alignmentRadius, cohesionRadius));

			BoostRTree rTree; MamziIndex mamziIndex;

			if (indexingMethod == 0) {
				mamziIndex = MamziIndex(particles, maxRadius);
			} else {
				rTree = BoostRTree(particles);
			}


			for (size_t i = 0; i < particles.size(); i++) {
				auto &p = particles[i];

				std::vector<int> neighbors;
				if (indexingMethod == 0) {
					neighbors = mamziIndex.getNeighbours(p.position, maxRadius);
				} else {
					neighbors = rTree.getNeighbours(p.position, maxRadius);
				}

//			std::cout<<neighbors.size()<<std::endl;
				for (auto &j: neighbors) {
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
					if (neighbor.isBarrier) {
						continue;
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
		};

		vec3f calculateSeparationForce(Particle& a, Particle& b) const {
			auto dist = glm::length(a.position - b.position);
			return separationConstant * (a.position - b.position) / (dist * dist);
		};

		vec3f calculateAlignmentForce(Particle& a, Particle& b) const {
			return alignmentConstant * (b.velocity - a.velocity);
		};

		vec3f calculateCohesionForce(Particle& a, Particle& b) const {
			return cohesionConstant * (b.position - a.position);
		};


		float bounds = 30.f,
				velocityLimit = 3.f,
				turnFactor = 0.2f,
				separationRadius = 2.f,
				separationAngle = M_PI * (1.f/2.f),
				separationConstant = 1.f,

				alignmentRadius = 4.f,
				alignmentAngle = M_PI,
				alignmentConstant = 1.f,

				cohesionRadius = 3.f,
				cohesionAngle = M_PI,
				cohesionConstant = 1.0f;

	};

//
// Particle Model
//
	class ParticleModel : public Model {
	public:
		explicit ParticleModel(int);
		void reset(int) override;
	};

	class ParticleBarrierModel: public Model {
	public:
		ParticleBarrierModel();
		void reset(int) override;
	};
} // namespace simulation
