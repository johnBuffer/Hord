#pragma once

#include "physic_objects.hpp"


struct Utils
{
	static float dot(const std::vector<float>& v1, const std::vector<float>& v2)
	{
		float result = 0.0f;

		const uint64_t size = v1.size();
		for (uint64_t i(0); i < size; ++i) {
			result += v1[i] * v2[i];
		}

		return result;
	}

	static std::vector<float> plus(const std::vector<float>& v1, const std::vector<float>& v2)
	{
		const uint64_t size = v1.size();
		std::vector<float> result(size);
		for (uint64_t i(0); i < size; ++i) {
			result[i] = v1[i] + v2[i];
		}

		return result;
	}

	static std::vector<float> mult(const std::vector<float>& v1, const std::vector<float>& v2)
	{
		const uint64_t size = v1.size();
		std::vector<float> result(size);
		for (uint64_t i(0); i < size; ++i) {
			result[i] = v1[i] * v2[i];
		}

		return result;
	}

	static std::vector<float> mult(float f, const std::vector<float>& v)
	{
		const uint64_t size = v.size();
		std::vector<float> result(size);
		for (uint64_t i(0); i < size; ++i) {
			result[i] = f * v[i];
		}

		return result;
	}
};

// Maybe use double ?
struct AtomContact
{
	Atom* atom_a;
	Atom* atom_b;

	float accumulated_lambda;
	float bias;

	const float friction = 0.05f;

	std::vector<float> j;
	std::vector<float> j_friction;
	std::vector<float> inv_m;

	Vec2 impulse;
	Vec2 contact_point;
	Vec2 contact_normal;

	AtomContact()
		: atom_a(nullptr)
		, atom_b(nullptr)
		, accumulated_lambda(0.0f)
	{}

	AtomContact(Atom* a, Atom* b)
		: atom_a(a)
		, atom_b(b)
		, accumulated_lambda(0.0f)
		, j(6)
		, j_friction(6)
		, inv_m(6)
	{}

	float getDelta2() const
	{
		return (atom_a->position - atom_b->position).getLength() - 2 * atom_a->radius;
	}

	bool isValid() const
	{
		// Need to adapt to other normal
		return getDelta2() < 0.0f;
	}

	Vec2 getContactPointA(const Vec2& collision_vec) const
	{
		return atom_a->position.minus(collision_vec * atom_a->radius);
	}

	Vec2 getContactPointB(const Vec2& collision_vec) const
	{
		return atom_b->position.plus(collision_vec * atom_b->radius);
	}

	void initialize()
	{
		const Vec2 collision_vec = atom_a->position - atom_b->position;
		const float distance = collision_vec.getLength();
		contact_normal = collision_vec.getNormalized();

		contact_point = getContactPointA(contact_normal);
		const Vec2 to_contact_point_a = contact_point - atom_a->parent->center_of_mass;
		const Vec2 to_contact_point_b = getContactPointB(contact_normal) - atom_b->parent->center_of_mass;
		// Normal
		j[0] = contact_normal.x;
		j[1] = contact_normal.y;
		j[2] = to_contact_point_a.cross(contact_normal);
		j[3] = -contact_normal.x;
		j[4] = -contact_normal.y;
		j[5] = to_contact_point_b.cross(contact_normal);
		// Friction
		const Vec2 contact_tangent = contact_normal.getNormal();
		j_friction[0] = contact_tangent.x;
		j_friction[1] = contact_tangent.y;
		j_friction[2] = to_contact_point_a.cross(contact_tangent);
		j_friction[3] = -contact_tangent.x;
		j_friction[4] = -contact_tangent.y;
		j_friction[5] = to_contact_point_b.cross(contact_tangent);
		// Intertia
		const float inv_mass_a = 1.0f / atom_a->parent->getMass();
		const float inv_moment_intertia_a = 1.0f / atom_a->parent->getMomentInertia();
		const float inv_mass_b = 1.0f / atom_b->parent->getMass();
		const float inv_moment_intertia_b = 1.0f / atom_b->parent->getMomentInertia();
		inv_m[0] = inv_mass_a;
		inv_m[1] = inv_mass_a;
		inv_m[2] = inv_moment_intertia_a;
		inv_m[3] = inv_mass_b;
		inv_m[4] = inv_mass_b;
		inv_m[5] = inv_moment_intertia_b;

		const float delta = 2.0f * atom_a->radius - distance;
		const float c = Vec2(0.0f, delta).dot(contact_normal);
		bias = 0.2f / 0.016f * ((c < 0.0f) ? c : 0.0f);

		accumulated_lambda = 0.0f;
	}

	void computeImpulse()
	{
		const Vec2 body_1_velocity = atom_a->parent->getVelocity();
		const Vec2 body_2_velocity = atom_b->parent->getVelocity();
		std::vector<float> v = {
			body_1_velocity.x,
			body_1_velocity.y,
			atom_a->parent->getAngularVelocity(),
			body_2_velocity.x,
			body_2_velocity.y,
			atom_b->parent->getAngularVelocity()
		};

		// Normal
		// var lambda = -(MV.dot(J[j], v) + bias[j]) / MV.dot(J[j], MV.VxV(MInv[j], J[j]));
		float lambda = -(Utils::dot(j, v) + bias) / Utils::dot(j, Utils::mult(inv_m, j));
		if (accumulated_lambda + lambda < 0) {
			lambda = -accumulated_lambda;
		}
		accumulated_lambda += lambda;
		impulse = contact_normal * accumulated_lambda;
		v = Utils::plus(v, Utils::mult(inv_m, Utils::mult(lambda, j)));
		atom_a->parent->velocity         = Vec2(v[0], v[1]);
		atom_a->parent->angular_velocity = v[2];
		atom_b->parent->velocity         = Vec2(v[3], v[4]);
		atom_b->parent->angular_velocity = v[5];

		// Friction
		// var lambdaFriction = -(MV.dot(Jt[j], v) + 0 * bias[j]) / MV.dot(Jt[j], MV.VxV(MInv[j], Jt[j]));
		float lambda_friction = -Utils::dot(j_friction, v) / Utils::dot(j_friction, Utils::mult(inv_m, j_friction));
		if (lambda_friction > friction * lambda) {
			lambda_friction = friction * lambda;
		}
		else if (lambda_friction < -friction * lambda) {
			lambda_friction = -friction * lambda;
		}
		//v = MV.VpV(v, MV.VxV(MInv[j], MV.SxV(lambdaFriction, Jt[j])));
		v = Utils::plus(v, Utils::mult(inv_m, Utils::mult(lambda_friction, j_friction)));
		atom_a->parent->velocity         = Vec2(v[0], v[1]);
		atom_a->parent->angular_velocity = v[2];
		atom_b->parent->velocity         = Vec2(v[3], v[4]);
		atom_b->parent->angular_velocity = v[5];
	}
};
