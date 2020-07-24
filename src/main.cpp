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

    const uint32_t agents_count = 10;
    std::vector<Agent> agents;
    for (uint32_t i(agents_count); i--;) {
        agents.emplace_back(rand()%WinWidth, rand() % WinHeight);
    }

    /*Solver solver;
    for (uint32_t i(agents_count); i--;) {
        solver.atoms.emplace_back();
    }*/

    ComposedObject object;

    const float atom_radius = 12.0f;
    for (uint32_t x(0); x < 5; ++x) {
        for (uint32_t y(0); y < 5; ++y) {
            object.addAtom(Vec2(150.0f + x * 2.0f * atom_radius, 70.0f + y * 2.0f * atom_radius));
        }
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
        const sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);
        end.x = mouse_pos.x;
        end.y = mouse_pos.y;
    });

    const float dt = 0.008f;

    while (window.isOpen()) {
        display_manager.processEvents();
        const sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);

        object.update(dt);

        window.clear(sf::Color::Black);

        const sf::RenderStates rs = display_manager.getRenderStates();

        Renderer::renderAtoms(window, object, rs);

		window.display();
    }

    return 0;
}
