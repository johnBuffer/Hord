#pragma once

#include "physic_objects.hpp"


struct HorizontalBoundaryContact
{
	Atom* atom;
	HorizontalBoundary* boundary;

	float accumulated_lambda;
	Vec2 j_linear_velocity;
	float j_angular_velocity;

	HorizontalBoundaryContact()
		: atom(nullptr)
		, boundary(nullptr)
		, accumulated_lambda(0.0f)
		, j_linear_velocity()
		, j_angular_velocity(0.0f)
	{}

	HorizontalBoundaryContact(Atom* a, HorizontalBoundary* b)
		: atom(a)
		, boundary(b)
		, accumulated_lambda(0.0f)
		, j_linear_velocity()
		, j_angular_velocity(0.0f)
	{}

	float getDelta() const
	{
		return (atom->position.y + atom->radius - boundary->coord);
	}

	bool isValid() const
	{
		// Need to adapt to other normal
		return getDelta() > 0.0f;
	}

	Vec2 getContactPoint() const
	{
		return Vec2(atom->position.x, boundary->coord);
	}

	void initialize()
	{
		const Vec2& contact_normal = boundary->normal;
		j_linear_velocity = contact_normal;
		// Not sure about the sign
		const Vec2 to_contact_point = atom->position.plus(Vec2(0.0f, atom->radius)) - atom->parent->center_of_mass;
		j_angular_velocity = to_contact_point.cross(contact_normal);
		const float c = Vec2(0.0f, getDelta()).dot(contact_normal);
		accumulated_lambda = 0.0f;
	}

	void computeImpulse()
	{
		const float inv_mass = 1.0f / atom->parent->getMass();
		const float inv_moment_intertia = 1.0f / atom->parent->getMomentInertia();

		const float nom = (j_linear_velocity.dot(atom->parent->velocity) + j_angular_velocity * atom->parent->angular_velocity);
		const float denom = j_linear_velocity.dot(Vec2(inv_mass, inv_mass)) + j_angular_velocity * inv_moment_intertia;

		// Maybe bias
		float lambda = nom / denom;

		if (accumulated_lambda + lambda < 0) {
			lambda = -accumulated_lambda;
		}

		accumulated_lambda += lambda;

		//v = MV.VpV(v, MV.VxV(MInv[j], MV.SxV(lambda, J[j])));
		const Vec2 linear_impulse = atom->parent->velocity.plus(j_linear_velocity * (inv_mass * lambda));
		const float angular_impulse = atom->parent->angular_velocity + j_angular_velocity * inv_moment_intertia * lambda;

		atom->parent->velocity = linear_impulse;
		atom->parent->angular_velocity = angular_impulse;
	}
};


/*struct AtomContact
{
	Atom* atom_a;
	Atom* atom_b;

	float accumulated_lambda;
	Vec2 j_linear_velocity_a;
	float j_angular_velocity_a;

	Vec2 j_linear_velocity_b;
	float j_angular_velocity_b;

	AtomContact()
		: atom_a(nullptr)
		, atom_b(nullptr)
		, accumulated_lambda(0.0f)
		, j_linear_velocity_a()
		, j_angular_velocity_a(0.0f)
		, j_linear_velocity_b()
		, j_angular_velocity_b(0.0f)
	{}

	AtomContact(Atom* a, Atom* b)
		: atom_a(a)
		, atom_b(b)
		, accumulated_lambda(0.0f)
		, j_linear_velocity_a()
		, j_angular_velocity_a(0.0f)
		, j_linear_velocity_b()
		, j_angular_velocity_b(0.0f)
	{}

	float getDelta2() const
	{
		return (atom_a->position - atom_b->position).getLength2() - atom_a->radius * atom_a->radius;
	}

	bool isValid() const
	{
		// Need to adapt to other normal
		return getDelta2() < 0.0f;
	}

	Vec2 getContactPointA() const
	{
		
	}

	Vec2 getContactPointB() const
	{

	}

	void initialize()
	{
		const Vec2 collision_vec = atom_a->position - atom_b->position;
		const float distance = collision_vec.getLength();

		const Vec2 contact_normal = collision_vec.getNormalized();
		j_linear_velocity_a = contact_normal;
		j_linear_velocity_b = -contact_normal;
		// Not sure about the sign
		const Vec2 to_contact_point = getContactPoint() - atom->parent->center_of_mass;
		j_angular_velocity = to_contact_point.cross(contact_normal);
		const float c = Vec2(0.0f, getDelta()).dot(contact_normal);
		accumulated_lambda = 0.0f;
	}

	void computeImpulse()
	{
		const float inv_mass = 1.0f / atom->parent->getMass();
		const float inv_moment_intertia = 1.0f / atom->parent->getMomentInertia();

		const float nom = (j_linear_velocity.dot(atom->parent->velocity) + j_angular_velocity * atom->parent->angular_velocity);
		const float denom = j_linear_velocity.dot(Vec2(inv_mass, inv_mass)) + j_angular_velocity * inv_moment_intertia;

		// Maybe bias
		float lambda = nom / denom;

		if (accumulated_lambda + lambda < 0) {
			lambda = -accumulated_lambda;
		}

		accumulated_lambda += lambda;

		//v = MV.VpV(v, MV.VxV(MInv[j], MV.SxV(lambda, J[j])));
		const Vec2 linear_impulse = atom->parent->velocity.plus(j_linear_velocity * (inv_mass * lambda));
		const float angular_impulse = atom->parent->angular_velocity + j_angular_velocity * inv_moment_intertia * lambda;

		atom->parent->velocity = linear_impulse;
		atom->parent->angular_velocity = angular_impulse;
	}
};*/
