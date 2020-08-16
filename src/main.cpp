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
	constexpr uint32_t WinWidth  = 1600;
	constexpr uint32_t WinHeight = 900;

    sf::RenderWindow window(sf::VideoMode(WinWidth, WinHeight), "Hord", sf::Style::Default, settings);
    window.setVerticalSyncEnabled(true);

    Grid grid(20, 80, 45);
    sf::Vector2f start(0.0f, 0.0f);
    sf::Vector2f end(10.0f, 10.0f);

    Solver solver;

    bool pause = false;
    bool step = false;
    const float atom_radius = 8.0f;

    solver.objects.emplace_back();
    solver.objects.back().moving = false;
    for (uint32_t x(0); x < WinWidth / (2.0f * atom_radius); ++x) {
        solver.objects.back().addAtom(Vec2(0.0f + x * 2.0f * atom_radius, WinHeight));
    }
    for (uint32_t x(0); x < WinWidth / (2.0f * atom_radius); ++x) {
        solver.objects.back().addAtom(Vec2(0.0f + x * 2.0f * atom_radius, 0.0f));
    }
    for (uint32_t y(0); y < WinHeight / (2.0f * atom_radius); ++y) {
        solver.objects.back().addAtom(Vec2(0.0f, y * 2.0f * atom_radius));
    }
    for (uint32_t y(0); y < WinHeight / (2.0f * atom_radius); ++y) {
        solver.objects.back().addAtom(Vec2(WinWidth, y * 2.0f * atom_radius));
    }

	DisplayManager display_manager(window);
    display_manager.event_manager.addKeyPressedCallback(sf::Keyboard::C, [&](const sf::Event& ev) {
        const sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);
        grid.setCellAtWorld(sf::Vector2f(mouse_pos.x, mouse_pos.y), 1);
    });

    display_manager.event_manager.addKeyPressedCallback(sf::Keyboard::S, [&](const sf::Event& ev) {
        const sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);
        start.x = mouse_pos.x;
        start.y = mouse_pos.y;
    });

    display_manager.event_manager.addKeyPressedCallback(sf::Keyboard::E, [&](const sf::Event& ev) {
        solver.objects.emplace_back();
        solver.objects.back().angular_velocity = 2.0f;
        for (uint32_t x(0); x < 5; ++x) {
            for (uint32_t y(0); y < 5; ++y) {
                solver.objects.back().addAtom(Vec2(800.0f + x * 2.0f * atom_radius, 370.0f + y * 2.0f * atom_radius));
            }
        }
    });

    display_manager.event_manager.addKeyPressedCallback(sf::Keyboard::Space, [&](const sf::Event& ev) {
        step = true;
    });

    display_manager.event_manager.addKeyPressedCallback(sf::Keyboard::P, [&](const sf::Event& ev) {
        pause = !pause;
    });

    const float dt = 0.016f;

    while (window.isOpen()) {
        display_manager.processEvents();
        const sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);

        if (step || !pause) {
            solver.update(dt);
            step = false;
        }

        window.clear(sf::Color::Black);

        const sf::RenderStates rs = display_manager.getRenderStates();

        for (ComposedObject& o : solver.objects) {
            Renderer::renderAtoms(window, o, rs);
        }

        Renderer::renderContacts(window, solver.atom_contacts, rs);

		window.display();
    }

    return 0;
}
