#pragma once
#include <vector>
#include "vec.hpp"


struct Atom
{
	Atom()
		: position()
		, delta_position()
		, acceleration()
		, mass(1.0f)
	{}

	Atom(const Vec2& p)
		: position(p.x, p.y)
		, delta_position()
		, acceleration()
		, mass(1.0f)
	{}

	float getDistance2With(const Atom& other) const
	{
		return (position - other.position).getLength();
	}

	void move(const Vec2& v)
	{
		delta_position += v;
	}

	void moveTo(const Vec2& p)
	{
		const Vec2 v = p - position;
		move(v);
	}

	void update(float dt)
	{

	}

	void reset()
	{
		delta_position = {};
		acceleration = {};
	}

	Vec2 position;
	Vec2 delta_position;
	Vec2 acceleration;
	float mass;
	float radius = 12.0f;
};


struct ComposedObject
{
	ComposedObject()
		: center_of_mass()
		, speed()
		, angular_speed(0.0f)
		, angle(0.0f)
		, applied_force(0.0f, 0.0f)
	{}

	void addAtom(const Vec2& p)
	{
		atoms.emplace_back(p);
	}

	void computeCenterOfMass()
	{
		Vec2 com;
		for (const Atom& a : atoms) {
			com += a.position;
		}
		center_of_mass = com / float(atoms.size());
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
		return float(atoms.size());
	}

	void update(float dt)
	{
		// Stuff
		const float mass = 1.0f / getMass();
		speed += (applied_force / mass) * dt;
		// Reset
		applied_force = Vec2(0.0f, 0.0f);
	}

	void updateState(float dt)
	{
		translate(speed * dt);
		center_of_mass += speed * dt;

		rotate(angular_speed * dt);
		angle += angular_speed * dt;
	}

	void solveCollisions()
	{

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

	std::vector<Atom> atoms;
	Vec2 center_of_mass;
	Vec2 speed;
	Vec2 applied_force;

	float angular_speed;
	float angle;
};
