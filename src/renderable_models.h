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

		static Cylinder c1(Point1(0.f, 0.f, 0.f), Point2(0.05f, 0.f, 0.f), Radius(model.bounds), AzimuthPoints(50));
		static Cylinder c2(Point1(0.f, 0.f, 0.f), Point2(0.f, 0.05f, 0.f), Radius(model.bounds), AzimuthPoints(50));
		static Cylinder c3(Point1(0.f, 0.f, 0.f), Point2(0.f, 0.f, 0.05f), Radius(model.bounds), AzimuthPoints(50));

		static auto circle_renderable1 = createRenderable(
				c1,
				Phong(Colour(0.1f, 0.1f, 0.1f),
					  LightPosition(100.f, 100.f, 100.f)) // style
		);
		static auto circle_renderable2 = createRenderable(
				c2,
				Phong(Colour(0.1f, 0.1f, 0.1f),
					  LightPosition(100.f, 100.f, 100.f)) // style
		);
		static auto circle_renderable3 = createRenderable(
				c3,
				Phong(Colour(0.1f, 0.1f, 0.1f),
					  LightPosition(100.f, 100.f, 100.f)) // style
		);



		// draw the renderable
		draw(point_renders, view);
		draw(circle_renderable1, view);
		draw(circle_renderable2, view);
		draw(circle_renderable3, view);
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
