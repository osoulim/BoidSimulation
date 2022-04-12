#pragma once

#include "givr.h"

#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

#include "models.h"

using namespace glm;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

namespace simulation {

//
// Particle Model
//
	template <typename View>
	void render(ParticleModel const &model, View const &view) {
		// static: only initiallized once, the first time this function is called.
		// This isn't the most elegant method to do this but it works. Just put your
		// draw calls in here.

		auto boidGeometry = TriangleSoup();
		boidGeometry.push_back(Triangle(Point1(0.f, 1.f, 0.15f), Point2(-0.25, 0.f, 0.f), Point3(0.25, 0.f, 0.f)));
		boidGeometry.push_back(Triangle(Point1(0.f, 1.f, 0.15f), Point2(0.25, 0.f, 0.f), Point3(0.0, 0.f, 0.45f)));
		boidGeometry.push_back(Triangle(Point1(0.f, 1.f, 0.15f), Point2(0.0, 0.f, 0.45f), Point3(-0.25, 0.f, 0.f)));

		static auto point_renders = createInstancedRenderable(
				boidGeometry,
				NoShading(Colour(1.f, 0.4f, 0.2f)) // style
		);

		// load up renderable
		auto const &particles = model.particles;

		for (auto const &particle : particles) {
			auto newY = normalize(particle.velocity);
			auto newZ = cross(newY, givr::vec3f{0.f, 0.f, 1.f});
			auto newX = cross(newY, newZ);
			auto M = translate(mat4f{1.f}, particle.position)
					* mat4(vec4{newX, 0.f}, vec4{newY, 0.f}, vec4{newZ, 0.f}, vec4{vec3{0.f}, 1.f});
			addInstance(point_renders, M);
		}

		static auto grid = MultiLine();
		static int numberOfCells = 4;
		float cellSize = 2 * model.bounds / (numberOfCells-1);

		static std::vector<std::vector<int>> neighbors {
				std::vector<int> {-1, 0, 0},
				std::vector<int> {0, -1, 0},
				std::vector<int> {0, 0, -1},
		};

		for (int x = 0; x < numberOfCells; x++)
			for (int y = 0; y < numberOfCells; y++)
				for (int z = 0; z < numberOfCells; z++)
					for (auto &neighbor: neighbors) {
						auto newX = x + neighbor[0];
						auto newY = y + neighbor[1];
						auto newZ = z + neighbor[2];
						if (newX >= 0 && newX < numberOfCells && newY >= 0 && newY < numberOfCells && newZ >= 0 && newZ < numberOfCells) {
							grid.push_back(Line(Point1(x * cellSize - model.bounds, y * cellSize - model.bounds, z * cellSize - model.bounds),
												Point2(newX * cellSize - model.bounds, newY * cellSize - model.bounds, newZ * cellSize - model.bounds)));
						}
					}

		static auto grid_renderable = createRenderable(grid, NoShading(Colour(0.f, 0.f, 0.f)));

		// draw the renderable
		draw(point_renders, view);
		draw(grid_renderable, view);
	}

//
// Helper class/functions
//
	template <typename View> struct RenderableModel {

		template <typename Model>
		RenderableModel(Model const &model) : m_self(new model_t<Model>(model)) {}

		friend void render(RenderableModel const &renderable, View const &view) {
			renderable.m_self->renderSelf(view);
		}

		struct concept_t {
			virtual ~concept_t() = default;
			virtual void renderSelf(View const &view) const = 0;
		};

		template <typename Model> struct model_t : public concept_t {
			model_t(Model const &model) : data(model) {}
			void renderSelf(View const &view) const override { render(data, view); }
			Model const &data;
		};

		std::shared_ptr<concept_t const> m_self;
	};

	template <typename Model, typename View>
	RenderableModel<View> makeModelRenderable(Model const &model,
											  View const &view) {
		return RenderableModel<View>(model);
	}

} // namespace simulation
