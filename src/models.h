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

	struct Model {
		virtual ~Model() = default;
		virtual void reset(int) = 0;
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

		void applyVelocityLimit(float limit) {
			auto velocityAmount = glm::length(velocity);
			if (velocityAmount > limit) {
				velocity = velocity / velocityAmount * limit;
			}
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
		void reset(int) override;
		void step(float dt) override;

		vec3f calculateSeparationForce(Particle& a, Particle& b) const;
		vec3f calculateAlignmentForce(Particle& a, Particle& b) const;
		vec3f calculateCohesionForce(Particle& a, Particle& b) const;

	public:
		std::vector<Particle> particles;
		float bounds = 30.f,
			velocityLimit = 3.f,
			turnFactor = 0.2f,
			separationRadius = 2.f,
			separationAngle = M_PI * (3.f/4.f),
			separationConstant = 1.f,

			alignmentRadius = 4.f,
			alignmentAngle = M_PI,
			alignmentConstant = 1.f,

			cohesionRadius = 3.f,
			cohesionAngle = M_PI,
			cohesionConstant = 1.0f;
	};

	struct SpatialStructure {
		virtual ~SpatialStructure() = default;
		virtual std::vector<int> getNeighbours(vec3f, float) = 0;
	};

	class BoostRTree: SpatialStructure {
	public:
		std::vector<int> getNeighbours(vec3f, float) override;
		explicit BoostRTree(std::vector<Particle> const &);

	private:
		std::unique_ptr<bgi::rtree<IndexedPoint , bgi::quadratic<16>>> tree;
	};

} // namespace simulation
