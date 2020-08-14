#pragma once
#include <vector>
#include "physic_objects.hpp"


struct Solver
{
	void solveCollisions()
	{
		
	}

	void solveBoundaryCollisions()
	{
		for (ComposedObject& o : objects) {
			for (Atom& a : o.atoms) {
				Vec2 p = a.position;
				Vec2 contact_point;
				Vec2 normal;
				
				if (a.position.x > boundaries_max.x - a.radius) {
					p.x = boundaries_max.x - a.radius;
				}
				else if (a.position.x < boundaries_min.x + a.radius) {
					p.x = boundaries_min.x + a.radius;
				}

				if (a.position.y > boundaries_max.y - a.radius) {
					p.y = boundaries_max.y - a.radius;
				}
				else if (a.position.y < boundaries_min.y + a.radius) {
					p.y = boundaries_min.y + a.radius;
				}
				a.moveTo(p);
			}
		}
	}

	void applyGravity()
	{
		const Vec2 gravity(0.0f, 10.0f);
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

		solveBoundaryCollisions();

		for (ComposedObject& o : objects) {
			o.updateState(dt);
		}

		/*solveCollisions();

		for (ComposedObject& o : objects) {
			o.solveCollisions();
		}*/
	}

	std::vector<ComposedObject> objects;
	const Vec2 boundaries_min = Vec2(50.0f, 50.0f);
	const Vec2 boundaries_max = Vec2(1550.0f, 850.0f);
};
