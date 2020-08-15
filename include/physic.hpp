#pragma once
#include <vector>
#include <list>
#include "physic_objects.hpp"
#include "contact.hpp"


struct Solver
{
	void solveCollisions()
	{
		
	}

	void addBoundary(const Vec2& normal, float coord)
	{
		boundaries.emplace_back(normal, coord);
	}

	void solveBoundaryCollisions(float dt)
	{
		//contacts.remove_if([&](const HorizontalBoundaryContact& c) { return c.isValid(); });
		contacts.clear();

		for (ComposedObject& o : objects) {
			for (Atom& a : o.atoms) {
				HorizontalBoundaryContact contact(&a, &boundaries.front());
				if (contact.isValid()) {
					contact.initialize();
					contacts.push_back(contact);
					//contact.computeImpulse();
				}
			}
		}

		const uint32_t iterations_count = 4;
		for (uint32_t i(iterations_count); i--;) {
			for (HorizontalBoundaryContact& c : contacts) {
				c.computeImpulse();
			}
		}
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

		solveBoundaryCollisions(dt);

		for (ComposedObject& o : objects) {
			o.updateState(dt);
		}
	}

	std::list<ComposedObject> objects;
	std::list<HorizontalBoundary> boundaries;
	std::list<HorizontalBoundaryContact> contacts;

	const Vec2 boundaries_min = Vec2(50.0f, 50.0f);
	const Vec2 boundaries_max = Vec2(1550.0f, 850.0f);
};
