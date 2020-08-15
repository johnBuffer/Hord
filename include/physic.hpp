#pragma once
#include <vector>
#include "physic_objects.hpp"
#include "constraint.hpp"


struct Solver
{
	void solveCollisions()
	{
		
	}

	void addBoundary(float c, float d, BoundaryConstraint::Type t)
	{
		boundaries.emplace_back(c, d, t);
	}

	void solveBoundaryCollisions(float dt)
	{
		for (ComposedObject& o : objects) {
			for (Atom& a : o.atoms) {
				for (const BoundaryConstraint& b : boundaries) {
					if (b.needCorrection(a)) {
						applyImpulse(o, b.getImpulse(a), a.position, dt);
					}
				}
			}
		}
	}

	void applyImpulse(ComposedObject& o, const Impulse& i, const Vec2& at, float dt)
	{
		o.applyImpulseAt(i.linear * dt, at);
	}

	void applyGravity()
	{
		const Vec2 gravity(0.0f, 100.0f);
		for (ComposedObject& o : objects) {
			o.accelerate(gravity);
		}
	}

	void update(float dt)
	{
		applyGravity();

		for (ComposedObject& o : objects) {
			o.update(dt);
		}

		const uint32_t iterations_count = 8;
		for (uint32_t i(iterations_count); i--;) {
			solveBoundaryCollisions(dt);
		}

		for (ComposedObject& o : objects) {
			o.updateState(dt);
		}
	}

	std::vector<ComposedObject> objects;
	std::vector<BoundaryConstraint> boundaries;

	const Vec2 boundaries_min = Vec2(50.0f, 50.0f);
	const Vec2 boundaries_max = Vec2(1550.0f, 850.0f);
};
