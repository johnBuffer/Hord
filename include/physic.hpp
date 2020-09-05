#pragma once
#include <vector>
#include <list>
#include "physic_objects.hpp"
#include "contact.hpp"
#include <set>


struct Solver
{
	Solver()
	{
		const uint32_t max_atoms_count = 5000;
		contacts_states.resize(max_atoms_count);
		for (std::vector<uint64_t>& v : contacts_states) {
			v.resize(max_atoms_count);
		}
	}

	bool isNewContact(uint64_t i, uint64_t k) const
	{
		return !(contacts_states[i][k] + contacts_states[k][i]);
	}

	void setContact(uint64_t i, uint64_t k)
	{
		contacts_states[i][k] = 1;
	}

	void removeContact(uint64_t i, uint64_t k)
	{
		contacts_states[i][k] = 0;
		contacts_states[k][i] = 0;
	}

	void updateContacts()
	{
		for (auto it = atom_contacts.begin(); it != atom_contacts.end(); ++it) {
			if (it->isValid(atoms)) {
				it->initialize_jacobians(atoms);
			}
			else {
				removeContact(it->id_a, it->id_b);
				it = atom_contacts.erase(it)--;
			}
		}
	}

	void findContacts()
	{
		// Check for persistence here
		atom_contacts.remove_if([&](AtomContact& c) { 
			if (c.isValid(atoms)) {
				c.initialize_jacobians(atoms);
				c.applyLastImpulse(atoms);
				return false;
			}
			else {
				removeContact(c.id_a, c.id_b);
				return true;
			}
		});
		
		const size_t atoms_count = atoms.size();
		for (uint64_t i(0); i < atoms_count; ++i) {
			for (uint64_t k(0); k < atoms_count; ++k) {
				if (isNewContact(i, k) && atoms[i].parent != atoms[k].parent) {
					AtomContact contact(i, k);
					if (contact.isValid(atoms)) {
						contact.initialize(atoms);
						atom_contacts.push_back(contact);
						setContact(i, k);
					}
				}
			}
		}
	}

	void applyGravity()
	{
		const Vec2 gravity(0.0f, 750.0f);
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

		findContacts();
		const uint32_t iterations_count = 8;
		for (uint32_t i(iterations_count); i--;) {
			for (AtomContact& c : atom_contacts) {
				c.computeImpulse(atoms);
			}
		}

		for (ComposedObject& o : objects) {
			o.updateState(dt, atoms);
		}

		//checkBroke();
	}

	void addAtomToLastObject(const Vec2& position, float mass=1.0f)
	{
		atoms.emplace_back(position);
		objects.back().addAtom(atoms.size() - 1, atoms);
	}

	void checkBroke()
	{
		const float threshold = 200.0f;
		for (const AtomContact& c : atom_contacts) {
			if (std::abs(c.lambda) > threshold) {
				breakObject(*atoms[c.id_a].parent, c);
				breakObject(*atoms[c.id_b].parent, c);
			}
		}
	}

	void breakObject(ComposedObject& object, const AtomContact& c)
	{
		uint32_t break_free_threshold = 40;
		if (!object.moving || object.break_free < break_free_threshold) {
			return;
		}

		object.break_free = 0;

		const Vec2 separation_vec = c.impulse.getNormal();

		uint64_t count = 0;
		for (uint64_t id : object.atoms_ids) {
			Atom& atom = atoms[id];
			if ((atom.position - c.contact_point).dot(separation_vec) < 0.0f) {
				++count;
			}
		}
		
		if (count > 1 && count < object.atoms_ids.size()) {
			std::list<uint64_t> to_remove;
			objects.emplace_back();
			ComposedObject& new_object = objects.back();
			for (uint64_t id : object.atoms_ids) {
				Atom& atom = atoms[id];
				if ((atom.position - c.contact_point).dot(separation_vec) < 0.0f) {
					new_object.addAtom(id, atoms);
					to_remove.push_back(id);
				}
			}

			const float ratio = count / float(object.atoms_ids.size());

			object.angular_velocity = object.angular_velocity * (1.0f - ratio);
			object.velocity = object.velocity * (1.0f - ratio);

			new_object.angular_velocity = object.angular_velocity * (ratio);
			new_object.velocity = object.velocity * (ratio);

			for (uint64_t id : to_remove) {
				object.removeAtom(id, atoms);
			}
		}
	}

	std::vector<Atom> atoms;
	std::list<ComposedObject> objects;
	std::list<AtomContact> atom_contacts;

	std::vector<std::vector<uint64_t>> contacts_states;

	const Vec2 boundaries_min = Vec2(50.0f, 50.0f);
	const Vec2 boundaries_max = Vec2(1550.0f, 850.0f);
};
