#pragma once
#include "physic_objects.hpp"


struct Impulse
{
	Impulse()
		: linear()
		, angular()
	{

	}

	Impulse(Vec2 l, float a)
		: linear(l)
		, angular(a)
	{

	}

	Vec2 linear;
	float angular;
};


struct Constraint
{
	virtual bool needCorrection(const Atom& atom) const = 0;
	virtual const Vec2 getContactPoint(const Atom& atom) const = 0;
	virtual Impulse getImpulse(const Atom& atom) const = 0;
};


struct BoundaryConstraint : public Constraint
{
	enum class Type {
		Horizontal = 0,
		Vertical = 1
	};
	
	BoundaryConstraint()
		: type(Type::Horizontal)
		, coord(0.0f)
		, direction(0.0f)
	{

	}

	BoundaryConstraint(float c, float d, Type t)
		: type(t)
		, coord(c)
		, direction(d)
	{

	}

	bool needCorrection(const Atom& atom) const override
	{
		const float delta = std::abs(coord - (type == Type::Horizontal ? atom.position.x : atom.position.y));
		return delta < atom.radius;
	}

	const Vec2 getContactPoint(const Atom& atom) const override
	{
		if (type == Type::Horizontal) {
			return Vec2(coord, atom.position.y);
		}
		return Vec2(atom.position.x, coord);
	}

	Impulse getImpulse(const Atom& atom) const override
	{
		const float dt = 0.016f;
		const Vec2 normal = (type == Type::Horizontal) ? Vec2(direction, 0.0f) : Vec2(0.0f, direction);
		
		const float inv_mass = 1.0f / atom.parent->getMass();
		const float inv_iner = 1.0f / atom.parent->getMomentInertia();
		const Vec2 to_contact_point = getContactPoint(atom) - atom.parent->center_of_mass;
		const float cross = to_contact_point.cross(normal);
		//const float cross2 = to_contact_point.getNormalized().cross(normal);

		const float m_eff = 1.0f / (inv_mass + inv_iner * cross * cross);
		const float c1_d = (atom.parent->velocity).plus(to_contact_point * (atom.parent->angular_velocity)).dot(normal);
		//const float c1_d = atom.getVelocity().dot(normal);
		const float lambda = c1_d * -m_eff;

		//const float cross2 = to_contact_point.getNormalized().cross(normal);

		Impulse result(normal * lambda, 0.0f * cross * lambda);

		return result;
	}

	Type type;
	float coord;
	float direction;
};
