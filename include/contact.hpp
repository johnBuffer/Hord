#pragma once

#include <index_vector.hpp>
#include "physic_objects.hpp"
#include "array.hpp"


struct Utils
{
	template<typename T, uint32_t N>
	static float dot(const Array<T, N>& v1, const Array<T, N>& v2)
	{
		float result = 0.0f;
		for (uint64_t i(0); i < N; ++i) {
			result += v1[i] * v2[i];
		}

		return result;
	}
	
	template<typename T, uint32_t N>
	static Array<T, N> plus(const Array<T, N>& v1, const Array<T, N>& v2)
	{
		Array<T, N> result;
		for (uint64_t i(0); i < N; ++i) {
			result[i] = v1[i] + v2[i];
		}

		return result;
	}

	template<typename T, uint32_t N>
	static void add(Array<T, N>& v1, const Array<T, N>& v2)
	{
		for (uint64_t i(0); i < N; ++i) {
			v1[i] += v2[i];
		}
	}

	template<typename T, uint32_t N>
	static Array<T, N> mult(const Array<T, N>& v1, const Array<T, N>& v2)
	{
		Array<T, N> result;
		for (uint64_t i(0); i < N; ++i) {
			result[i] = v1[i] * v2[i];
		}

		return result;
	}

	template<typename T, uint32_t N>
	static Array<T, N> mult(float f, const Array<T, N>& v)
	{
		Array<T, N> result;
		for (uint64_t i(0); i < N; ++i) {
			result[i] = f * v[i];
		}

		return result;
	}
};

struct AtomContact
{
	uint64_t id_a, id_b;

	float lambda;
	float accumulated_lambda;
	float bias;
	float delta;
	const float friction = 0.25f;

	Array<float, 6> j;
	Array<float, 6> j_friction;
	Array<float, 6> inv_m;
	Array<float, 6> v;

	Vec2 impulse;
	Vec2 contact_point;

	mutable float contact_length;
	mutable Vec2 contact_vec;
	mutable Vec2 contact_normal;

	uint32_t tick_count;

	AtomContact()
		: id_a(0)
		, id_b(0)
		, accumulated_lambda(0.0f)
		, tick_count(0)
	{}

	AtomContact(uint64_t a, uint64_t b)
		: id_a(a)
		, id_b(b)
		, accumulated_lambda(0.0f)
		, tick_count(0)
	{}

	float getDelta(const IndexVector<Atom>& atoms) const
	{
		contact_vec = atoms[id_a].position - atoms[id_b].position;
		contact_length = contact_vec.getLength();
		contact_normal = contact_vec / contact_length;
		return contact_length - 2 * atoms[id_a].radius;
	}

	// Needs to be done first, initializes contact vecs
	bool isValid(const IndexVector<Atom>& atoms)
	{
		delta = getDelta(atoms);
		return delta < 0.0f;
	}

	Vec2 getContactPointA(const Vec2& collision_vec, const Atom& atom) const
	{
		return atom.position.minus(collision_vec * atom.radius);
	}

	Vec2 getContactPointB(const Vec2& collision_vec, const Atom& atom) const
	{
		return atom.position.plus(collision_vec * atom.radius);
	}

	void initialize(const IndexVector<Atom>& atoms)
	{
		const Atom& atom_a = atoms[id_a];
		const Atom& atom_b = atoms[id_b];
		
		// Intertia
		const float inv_mass_a = 1.0f / atom_a.parent->getMass();
		const float inv_mass_b = 1.0f / atom_b.parent->getMass();
		inv_m[0] = inv_mass_a;
		inv_m[1] = inv_mass_a;
		inv_m[2] = 1.0f / atom_a.parent->getMomentInertia();
		inv_m[3] = inv_mass_b;
		inv_m[4] = inv_mass_b;
		inv_m[5] = 1.0f / atom_b.parent->getMomentInertia();

		// Jacobians
		initialize_jacobians(atoms);
	}

	void initialize_jacobians(const IndexVector<Atom>& atoms)
	{
		const Atom& atom_a = atoms[id_a];
		const Atom& atom_b = atoms[id_b];

		contact_point = getContactPointA(contact_normal, atom_a);
		const Vec2 to_contact_point_a = contact_point - atom_a.parent->center_of_mass;
		const Vec2 to_contact_point_b = getContactPointB(contact_normal, atom_b) - atom_b.parent->center_of_mass;
		// Normal
		j[0] = contact_normal.x;
		j[1] = contact_normal.y;
		j[2] = to_contact_point_a.cross(contact_normal);
		j[3] = -contact_normal.x;
		j[4] = -contact_normal.y;
		j[5] = -to_contact_point_b.cross(contact_normal);
		// Friction
		const Vec2 contact_tangent = contact_normal.getNormal();
		j_friction[0] = contact_tangent.x;
		j_friction[1] = contact_tangent.y;
		j_friction[2] = to_contact_point_a.cross(contact_tangent);
		j_friction[3] = -contact_tangent.x;
		j_friction[4] = -contact_tangent.y;
		j_friction[5] = -to_contact_point_b.cross(contact_tangent);

		accumulated_lambda = 0.0f;
	}

	void applyImpulse(Atom& atom_a, Atom& atom_b, const Array<float, 6>& impulse_vec)
	{
		atom_a.parent->velocity = Vec2(impulse_vec[0], impulse_vec[1]);
		atom_a.parent->angular_velocity = impulse_vec[2];
		atom_b.parent->velocity = Vec2(impulse_vec[3], impulse_vec[4]);
		atom_b.parent->angular_velocity = impulse_vec[5];
	}

	void applyImpulse(std::vector<Atom>& atoms, const Array<float, 6>& impulse_vec)
	{
		Atom& atom_a = atoms[id_a];
		Atom& atom_b = atoms[id_b];

		applyImpulse(atom_a, atom_b, impulse_vec);
	}

	void applyLastImpulse(IndexVector<Atom>& atoms)
	{
		++tick_count;

		Atom& atom_a = atoms[id_a];
		Atom& atom_b = atoms[id_b];

		const Vec2 body_1_velocity = atom_a.parent->getVelocity();
		const Vec2 body_2_velocity = atom_b.parent->getVelocity();

		Array<float, 6> v_tmp = {
			body_1_velocity.x,
			body_1_velocity.y,
			atom_a.parent->getAngularVelocity(),
			body_2_velocity.x,
			body_2_velocity.y,
			atom_b.parent->getAngularVelocity()
		};
		updateAccumulatedLambda();
		Utils::add(v_tmp, Utils::mult(inv_m, Utils::mult(lambda, j)));
		applyImpulse(atom_a, atom_b, v_tmp);
	}

	void updateAccumulatedLambda()
	{
		if (accumulated_lambda + lambda < 0) {
			lambda = -accumulated_lambda;
		}
		accumulated_lambda += lambda;
	}

	void computeImpulse(IndexVector<Atom>& atoms)
	{
		Atom& atom_a = atoms[id_a];
		Atom& atom_b = atoms[id_b];

		const Vec2 body_1_velocity = atom_a.parent->getVelocity();
		const Vec2 body_2_velocity = atom_b.parent->getVelocity();
		v = {
			body_1_velocity.x,
			body_1_velocity.y,
			atom_a.parent->getAngularVelocity(),
			body_2_velocity.x,
			body_2_velocity.y,
			atom_b.parent->getAngularVelocity()
		};

		const float dt = 0.016f;

		// Friction
		float lambda_friction = -Utils::dot(j_friction, v) / Utils::dot(j_friction, Utils::mult(inv_m, j_friction));
		if (lambda_friction > friction * lambda) {
			lambda_friction = friction * lambda;
		}
		else if (lambda_friction < -friction * lambda) {
			lambda_friction = -friction * lambda;
		}

		Utils::add(v, Utils::mult(inv_m, Utils::mult(lambda_friction, j_friction)));
		applyImpulse(atom_a, atom_b, v);

		// Normal
		const float bias_factor = 0.2f;
		bias = bias_factor / dt * std::min(delta, 0.0f);

		const float denom = Utils::dot(j, Utils::mult(inv_m, j));
		lambda = -(Utils::dot(j, v) + bias) / denom;
		updateAccumulatedLambda();

		impulse = contact_normal * lambda;
		Utils::add(v, Utils::mult(inv_m, Utils::mult(lambda, j)));
		applyImpulse(atom_a, atom_b, v);
	}

	void applyBiasImpulse(Atom& atom_a, Atom& atom_b, const Array<float, 6>& v_bias)
	{
		atom_a.parent->bias_velocity = Vec2(v_bias[0], v_bias[1]);
		atom_a.parent->angular_velocity = v_bias[2];
		atom_b.parent->bias_velocity = Vec2(v_bias[3], v_bias[4]);
		atom_b.parent->angular_velocity = v_bias[5];
	}
};
