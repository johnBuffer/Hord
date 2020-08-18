#pragma once
#include <vector>
#include <list>
#include "physic_objects.hpp"
#include "contact.hpp"
#include <set>


struct Solver
{
	void findContacts()
	{
		// Check for persistence here
		atom_contacts.clear();
		const size_t atoms_count = atoms.size();
		for (uint64_t i(0); i < atoms_count; ++i) {
			for (uint64_t k(i+1); k < atoms_count; ++k) {
				if (atoms[i].parent != atoms[k].parent) {
					AtomContact contact(&atoms[i], &atoms[k]);
					if (contact.isValid()) {
						contact.initialize();
						atom_contacts.push_back(contact);
					}
				}
			}
		}
	}

	void applyGravity()
	{
		const Vec2 gravity(0.0f, 980.0f);
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
		const uint32_t iterations_count = 16;
		for (uint32_t i(iterations_count); i--;) {
			for (AtomContact& c : atom_contacts) {
				c.computeImpulse();
			}
		}

		for (ComposedObject& o : objects) {
			o.updateState(dt, atoms);
		}
	}

	void addAtomToLastObject(const Vec2& position, float mass=1.0f)
	{
		atoms.emplace_back(position);
		objects.back().addAtom(atoms.size() - 1, atoms);
		atoms.back().parent = &objects.back();
	}

	std::vector<Atom> atoms;
	std::list<ComposedObject> objects;
	std::list<AtomContact> atom_contacts;

	const Vec2 boundaries_min = Vec2(50.0f, 50.0f);
	const Vec2 boundaries_max = Vec2(1550.0f, 850.0f);
};
