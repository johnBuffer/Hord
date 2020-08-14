#pragma once
#include "physic_objects.hpp"


struct Constraint
{
	virtual bool needCorrection(const Atom& atom) const {};
	virtual const Vec2 getContactPoint(const Atom& atom) const {}
	virtual float getDelta() const {}
};


struct BoundaryConstraint : public Constraint
{
	bool needCorrection(const Atom& atom) const override
	{
		const float delta = std::abs(coord - (type == Horizontal ? atom.position.x : atom.position.y));
		return delta < atom.radius;
	}

	const Vec2 getContactPoint(const Atom& atom) const override
	{

	}

	float getDelta() const override
	{

	}

	enum Type {
		Horizontal = 0,
		Vertical = 1
	};

	Type type;
	float coord;
	float direction;
};
