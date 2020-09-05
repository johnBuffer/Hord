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
		, break_free(0)
	{}

	void addAtom(uint64_t id, std::vector<Atom>& atoms)
	{
		atoms_ids.push_back(id);
		Atom& new_atom = atoms[id];
		new_atom.parent = this;
		if (!mass) {
			intertia = new_atom.mass;
		}
		else {
			addToInertia(new_atom);
		}
		mass += new_atom.mass;
		computeCenterOfMass(atoms);
	}

	void computeCenterOfMass(const std::vector<Atom>& atoms)
	{
		Vec2 com;
		for (uint64_t id : atoms_ids) {
			com += atoms[id].position;
		}
		center_of_mass = com / mass;
	}

	void addToInertia(const Atom& a)
	{
		const Vec2 r = center_of_mass - a.position;
		intertia += a.mass * r.getLength2();
	}

	void removeToInertia(const Atom& a)
	{
		const Vec2 r = center_of_mass - a.position;
		intertia -= a.mass * r.getLength2();
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

		++break_free;

		// Need to add moment
		velocity += (applied_force / mass) * dt;
		// Reset forces
		applied_force = Vec2(0.0f, 0.0f);
	}

	void updateState(float dt, std::vector<Atom>& atoms)
	{
		if (!moving) {
			return;
		}
		translate(velocity * dt, atoms);
		center_of_mass += velocity * dt;
		rotate(angular_velocity * dt, atoms);
		angle += angular_velocity * dt;
	}

	float getMomentInertia() const
	{
		return intertia;
	}

	void translate(const Vec2& v, std::vector<Atom>& atoms)
	{
		for (uint64_t a_id : atoms_ids) {
			atoms[a_id].position += v;
		}
	}

	void rotate(float r, std::vector<Atom>& atoms)
	{
		for (uint64_t a_id : atoms_ids) {
			atoms[a_id].position.rotate(center_of_mass, r);
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

	void removeAtom(uint64_t id, const std::vector<Atom>& atoms)
	{
		uint64_t i = 0;
		for (uint64_t a_id : atoms_ids) {
			if (a_id == id) {
				std::swap(atoms_ids.back(), atoms_ids[i]);
				atoms_ids.pop_back();

				mass -= atoms[id].mass;
				removeToInertia(atoms[id]);
				computeCenterOfMass(atoms);
				return;
			}
			++i;
		}
	}

	uint32_t break_free;

	std::vector<uint64_t> atoms_ids;
	Vec2 center_of_mass;
	Vec2 velocity;
	Vec2 applied_force;

	float angular_velocity;
	float angle;

	float mass;
	float intertia;

	bool moving;
};
