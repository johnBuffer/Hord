#pragma once
#include <vector>
#include <list>
#include "physic_objects.hpp"
#include "contact.hpp"
#include <set>
#include <index_vector.hpp>


struct Solver
{
	Solver()
		: frame_count(0)
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
		if (frame_count % 2) {
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
		else {
			for (uint64_t i(atoms_count); i--;) {
				for (uint64_t k(atoms_count); k--;) {
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

		checkBroke();
		++frame_count;
	}

	void addAtomToLastObject(const Vec2& position, float mass=1.0f)
	{
		atoms.emplace_back(position);
		objects.back().addAtom(atoms.size() - 1, atoms);
	}

	void addAtomTo(const Vec2& position, ComposedObject& object)
	{
		atoms.emplace_back(position);
		object.addAtom(atoms.size() - 1, atoms);
	}

	void checkBroke()
	{
		const float threshold = 700.0f;
		const float fragile_factor = 0.5f;
		for (const AtomContact& c : atom_contacts) {
			ComposedObject& object_1 = *atoms[c.id_a].parent;
			ComposedObject& object_2 = *atoms[c.id_b].parent;
			if (std::abs(c.lambda) > threshold) {
				breakObject(object_1, c);
				breakObject(object_2, c);
			}
			else {
				if (object_1.break_free < 2) {
					if (std::abs(c.lambda) > threshold * fragile_factor) {
						breakObject(object_1, c);
					}
				}

				if (object_2.break_free < 2) {
					if (std::abs(c.lambda) > threshold * fragile_factor) {
						breakObject(object_2, c);
					}
				}
			}
		}
	}

	void breakObject(ComposedObject& object, const AtomContact& c)
	{
		uint32_t break_free_threshold = 0;
		if (!object.moving) {
			return;
		}

		if (object.break_free > 2) {
			object.break_free = 0;
		}

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
			new_object.break_free = object.break_free;
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

	uint32_t frame_count;
	IndexVector<Atom> atoms;
	std::list<ComposedObject> objects;
	std::list<AtomContact> atom_contacts;

	std::vector<std::vector<uint64_t>> contacts_states;
};
