#include <SFML/Graphics.hpp>
#include <vector>
#include <list>
#include <cmath>
#include <iostream>
#include <fstream>

#include "display_manager.hpp"
#include "grid.hpp"
#include "render.hpp"
#include "agent.hpp"
#include "physic.hpp"


int main()
{
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
	constexpr uint32_t WinWidth  = 1920;
	constexpr uint32_t WinHeight = 1080;

    sf::RenderWindow window(sf::VideoMode(WinWidth, WinHeight), "Phys", sf::Style::Fullscreen, settings);
    window.setFramerateLimit(60);

    const float atom_radius = 8.0f;
    Grid grid(static_cast<uint64_t>(2.0f * atom_radius), WinWidth / static_cast<uint64_t>(2.0f * atom_radius), WinHeight / static_cast<uint64_t>(2.0f * atom_radius));
    sf::Vector2f start(0.0f, 0.0f);
    sf::Vector2f end(10.0f, 10.0f);

    Solver solver;

    bool pause = false;
    bool step = false;
    bool building = false;

    solver.objects.emplace_back();
    solver.objects.back().moving = false;
    for (uint32_t x(0); x < WinWidth / (2.0f * atom_radius); ++x) {
        solver.addAtomToLastObject(Vec2(0.0f + x * 2.0f * atom_radius, WinHeight));
    }
    /*for (uint32_t x(0); x < WinWidth / (2.0f * atom_radius); ++x) {
        solver.addAtomToLastObject(Vec2(0.0f + x * 2.0f * atom_radius, 0.0f));
    }
    for (uint32_t y(0); y < WinHeight / (2.0f * atom_radius); ++y) {
        solver.addAtomToLastObject(Vec2(0.0f, y * 2.0f * atom_radius));
    }
    for (uint32_t y(0); y < WinHeight / (2.0f * atom_radius); ++y) {
        solver.addAtomToLastObject(Vec2(WinWidth, y * 2.0f * atom_radius));
    }*/

	sf::Vector2i mouse_pos;

	DisplayManager display_manager(window);

    display_manager.event_manager.addKeyPressedCallback(sf::Keyboard::E, [&](const sf::Event& ev) {
        solver.objects.emplace_back();
        solver.objects.back().angular_velocity = -2.0f;
        uint32_t w = 5;
        uint32_t h = 5;
        for (uint32_t x(0); x < w; ++x) {
            for (uint32_t y(0); y < h; ++y) {
                solver.addAtomToLastObject(Vec2(mouse_pos.x + x * 2.0f * atom_radius + rand()%2, mouse_pos.y + y * 2.0f * atom_radius));
            }
        }
    });

    display_manager.event_manager.addKeyPressedCallback(sf::Keyboard::Space, [&](const sf::Event& ev) {
        step = true;
    });

    display_manager.event_manager.addKeyPressedCallback(sf::Keyboard::P, [&](const sf::Event& ev) {
        pause = !pause;
    });

    ComposedObject* current_object = nullptr;
    display_manager.event_manager.addKeyPressedCallback(sf::Keyboard::A, [&](const sf::Event& ev) {
        building = !building;

        if (building) {
            solver.objects.emplace_back();
            solver.objects.back().moving = false;
            current_object = &solver.objects.back();
        }
        else {
            grid.reset();
            if (solver.objects.back().atoms_ids.size()) {
                current_object->moving = true;
            }
        }
    });

    const float dt = 0.016f;

    while (window.isOpen()) {
        display_manager.processEvents();
        mouse_pos = sf::Mouse::getPosition(window);

        if (pause) {
            solver.objects.emplace_back();
            solver.addAtomToLastObject(Vec2(800.0f + rand() % 2, 350.0f));
        }

		//std::cout << solver.objects.size() << std::endl
        if (building) {
            const sf::Vector2f mouse_world_pos = sf::Vector2f(mouse_pos.x, mouse_pos.y);
            if (!grid.getCellContentAtWorld(mouse_world_pos)) {
                grid.setCellAtWorld(mouse_world_pos, 1);
                const sf::Vector2f atom_pos = grid.getDiscretizedCoords(mouse_world_pos);
                solver.addAtomTo(Vec2(atom_pos.x, atom_pos.y), *current_object);
            }
        }
        
		solver.update(dt);
		step = false;

        window.clear(sf::Color::Black);

        const sf::RenderStates rs = display_manager.getRenderStates();

        Renderer::renderAtoms(window, solver, rs);
        //Renderer::renderContacts(window, solver.atom_contacts, rs);

		window.display();
    }

    return 0;
}
