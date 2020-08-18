#pragma once
#include <vector>
#include "vec.hpp"


struct ComposedObject;


struct HorizontalBoundary
{
	HorizontalBoundary() = default;

	HorizontalBoundary(const Vec2& n, float c)
		: normal(n)
		, coord(c)
	{}

	Vec2 normal;
	float coord;
};


struct Atom
{
	Atom()
		: position()
		, last_position()
		, acceleration()
		, mass(1.0f)
		, parent(nullptr)
	{}

	Atom(const Vec2& p)
		: position(p)
		, last_position(p)
		, acceleration()
		, mass(1.0f)
		, parent(nullptr)
	{}

	void reset()
	{
		acceleration = {};
	}

	Vec2 getVelocity() const
	{
		return position - last_position;
	}

	ComposedObject* parent;
	Vec2 position, last_position;
	Vec2 acceleration;
	float mass;
	float radius = 8.0f;
};


struct ComposedObject
{
	ComposedObject()
		: center_of_mass()
		, velocity()
		, angular_velocity(0.0f)
		, angle(0.0f)
		, applied_force(0.0f, 0.0f)
		, intertia(0.0f)
		, mass(0.0f)
		, moving(true)
	{}

	void addAtom(const Vec2& p)
	{
		atoms.emplace_back(p);
		atoms.back().parent = this;

		if (!mass) {
			intertia = 1.0f;
		}
		else {
			addToInertia(atoms.back());
		}
		mass += 1.0f;
		computeCenterOfMass();
	}

	void computeCenterOfMass()
	{
		Vec2 com;
		for (const Atom& a : atoms) {
			com += a.position;
		}
		center_of_mass = com / mass;
	}

	void addToInertia(const Atom& a)
	{
		const Vec2 r = center_of_mass - a.position;
		intertia += a.mass * r.getLength2();
	}

	void applyForce(const Vec2& f)
	{
		applied_force += f;
	}

	void accelerate(const Vec2& a)
	{
		applyForce(a * getMass());
	}

	float getMass() const
	{
		return mass;
	}

	void update(float dt)
	{
		if (!moving) {
			return;
		}
		// Need to add moment
		velocity += (applied_force / mass) * dt;
		// Reset forces
		applied_force = Vec2(0.0f, 0.0f);
	}

	void updateState(float dt)
	{
		if (!moving) {
			return;
		}
		translate(velocity * dt);
		center_of_mass += velocity * dt;
		rotate(angular_velocity * dt);
		angle += angular_velocity * dt;
	}

	float getMomentInertia() const
	{
		return intertia;
	}

	void translate(const Vec2& v)
	{
		for (Atom& a : atoms) {
			a.position += v;
		}
	}

	void rotate(float r)
	{
		for (Atom& a : atoms) {
			a.position.rotate(center_of_mass, r);
		}
	}

	float getDistanceToCenterOfMass(const Vec2& p) const
	{
		return (center_of_mass - p).getLength();
	}

	Vec2 getVelocity() const
	{
		return velocity * float(moving);
	}

	float getAngularVelocity() const
	{
		return angular_velocity * float(moving);
	}

	Vec2 getAtomNextPosition(const Atom* a) const
	{
		const float dt = 0.016f;
		const Vec2 next_com = center_of_mass.plus(velocity * dt);
		Vec2 next_position = a->position.plus(velocity * dt);
		next_position.rotate(next_com, angular_velocity * dt);
		return next_position;
	}

	std::vector<Atom> atoms;
	Vec2 center_of_mass;
	Vec2 velocity;
	Vec2 applied_force;

	float angular_velocity;
	float angle;

	float mass;
	float intertia;

	bool moving;
};
