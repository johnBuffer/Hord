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
		atom_contacts.clear();
		uint32_t i = 0;
		for (ComposedObject& o1 : objects) {
			uint32_t k = 0;
			for (ComposedObject& o2 : objects) {
				if (k != i) {
					for (Atom& a1 : o1.atoms) {
						for (Atom& a2 : o2.atoms) {
							AtomContact contact(&a1, &a2);
							if (contact.isValid()) {
								contact.initialize();
								atom_contacts.push_back(contact);
							}
						}
					}
				}
				++k;
			}
			++i;
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
			o.updateState(dt);
		}
	}

	std::list<ComposedObject> objects;
	std::list<AtomContact> atom_contacts;

	std::map<Atom*, std::vector<Atom*>> contacts;

	const Vec2 boundaries_min = Vec2(50.0f, 50.0f);
	const Vec2 boundaries_max = Vec2(1550.0f, 850.0f);
};
