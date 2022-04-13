#include "models.h"

#include <iostream>

namespace simulation {

//
// Particle Model
//
	ParticleModel::ParticleModel(int inputBoidsNumber) {
		reset(inputBoidsNumber);
	}

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

	BoostRTree::BoostRTree(std::vector<Particle> const &particles) {
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
		tree = std::make_unique<bgi::rtree<IndexedPoint , bgi::quadratic<16>>>(indexedPoints);
	}

	std::vector<int> BoostRTree::getNeighbours(vec3f position, float maxRadius) {

		float x = position.x, y = position.y, z = position.z;
		Box queryBox(
				Point(x - maxRadius, y - maxRadius, z - maxRadius),
				Point(x + maxRadius, y + maxRadius, z + maxRadius)
		);

		std::vector<IndexedPoint> neighbors;
		tree->query(bgi::covered_by(queryBox), std::back_inserter(neighbors));

		std::vector<int> result;
		std::transform(
				neighbors.begin(), neighbors.end(),
				back_inserter(result),
				[](IndexedPoint indexedPoint) {
					return indexedPoint.second;
				}
		);

		return result;
	}

	MamziIndex::MamziIndex(const std::vector<Particle> & particles, float radius) {
		spatialMap.clear();

		int particleId = 0;
		for (auto &p: particles) {
			auto integerIndex = getIndex(p.position, radius);
			if (spatialMap.find(integerIndex) == spatialMap.end()) {
				spatialMap.insert(std::make_pair(integerIndex, std::vector<int>{}));
			}
			spatialMap[integerIndex].push_back(particleId);
			particleId++;
		}
	}

	std::vector<int> MamziIndex::getIndex(vec3f position, float radius) {
		return std::vector<int> {
			static_cast<int>(position.x / radius),
			static_cast<int>(position.y / radius),
			static_cast<int>(position.z / radius),
		};
	}

	std::vector<int> MamziIndex::getNeighbours(vec3f position, float radius) {
		auto result = std::vector<int>();
		auto integerIndex = getIndex(position, radius);

		for (int dx = -1; dx <= 1; dx++)
			for (int dy = -1; dy <= 1; dy++)
				for (int dz = -1; dz <= 1; dz++) {
					auto index = std::vector<int>{integerIndex[0] + dx, integerIndex[1] + dy, integerIndex[2] + dz};
					if (spatialMap.find(index) != spatialMap.end()) {
						auto tempVec = spatialMap[index];
						result.insert(result.end(), tempVec.begin(), tempVec.end());
					}
				}

		return result;
	}

	void ParticleBarrierModel::reset(int number) {
		particles.clear();
		for(int x = 0; x < 20; x++) {
			for (int y = -2; y <= 2; y++) {
				for (int z = -2; z <= 2; z++) {
					particles.push_back(Particle(vec3f{10.f + x * 1.f, y * 2.f, z * 2.f}, vec3f{-.2f, 0.f, 0.f}));
				}
			}
		}
		particles.push_back(Particle(vec3f{0.f}, vec3f{0.f}, true));
	}

	ParticleBarrierModel::ParticleBarrierModel() {
		reset(0);
	}
} // namespace simulation
