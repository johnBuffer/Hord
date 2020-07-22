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

    while (window.isOpen()) {
        display_manager.processEvents();
        const sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);

        for (Agent& a : agents) {
            a.update(sf::Vector2f(mouse_pos.x, mouse_pos.y), grid, 0.016f);
        }

        window.clear(sf::Color::Black);

        const sf::RenderStates rs = display_manager.getRenderStates();
        Renderer::renderGrid(window, grid, rs);

       /* const float r = 8.0f;
        sf::CircleShape c(r);
        c.setOrigin(r, r);
        c.setPosition(start);
        c.setFillColor(sf::Color::Blue);
        window.draw(c);

        const HitPoint ray = grid.castRayToPoint(start, end);
        c.setPosition(end);
        c.setFillColor(ray.hit ? sf::Color::Red : sf::Color::Green);
        window.draw(c);

        sf::VertexArray sight_va(sf::Lines, 2);
        sight_va[0].position = start;
        sight_va[1].position = end;
        sight_va[0].color = sf::Color::Red;
        sight_va[1].color = sf::Color::Red;
        window.draw(sight_va);*/

        for (const Agent& a : agents) {
            a.draw(window, rs);
        }

		window.display();
    }

    return 0;
}
