#pragma once
#include <vector>
#include "vec.hpp"


struct ComposedObject;


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

	float getDistance2With(const Atom& other) const
	{
		return (position - other.position).getLength();
	}

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
	float radius = 12.0f;
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
	{}

	void addAtom(const Vec2& p)
	{
		atoms.emplace_back(p);
		atoms.back().parent = this;

		if (!mass) {
			intertia = 1.0f;
		}
		else {
			addToInertia(p);
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

	void addToInertia(const Vec2& p)
	{
		const Vec2 r = center_of_mass - p;
		intertia += r.getLength2();
	}

	void applyForce(const Vec2& f)
	{
		applied_force += f;
	}

	void accelerate(const Vec2& a)
	{
		applyForce(a * getMass());
	}

	void applyImpulse(const Vec2& linear, float angular)
	{
		velocity += linear;
		angular_velocity += angular;
		//std::cout << "Linear: (" << linear.x << ", " << linear.y << ") angular: " << angular << std::endl;
	}

	void applyImpulseAt(const Vec2& linear, const Vec2& position)
	{
		//std::cout << "Linear: (" << linear.x << ", " << linear.y << ") angular: " << angular << std::endl;
		velocity += linear;
		const Vec2 to_point = position - center_of_mass;
		angular_velocity += linear.getLength() * to_point.cross(linear.getNormalized()) * 0.016f * 0.016f;
	}

	float getMass() const
	{
		return mass;
	}

	void update(float dt)
	{
		// Stuff
		velocity += (applied_force / mass) * dt;
		// Reset
		applied_force = Vec2(0.0f, 0.0f);
	}

	void updateState(float dt)
	{
		std::cout << "Vel: " << velocity.x << " " << velocity.y << " angular: " << angular_velocity * dt << std::endl;
		translate(velocity * dt);
		center_of_mass += velocity * dt;
		rotate(angular_velocity * dt);
		angle += angular_velocity * dt;
	}

	void solveCollisions()
	{

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

	std::vector<Atom> atoms;
	Vec2 center_of_mass;
	Vec2 velocity;
	Vec2 applied_force;

	float angular_velocity;
	float angle;

	float mass;
	float intertia;
};
