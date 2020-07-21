#include <SFML/Graphics.hpp>
#include <vector>
#include <list>
#include <cmath>
#include <iostream>
#include <fstream>

#include "display_manager.hpp"
#include "grid.hpp"
#include "render.hpp"


int main()
{
	sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
	constexpr uint32_t WinWidth  = 1600;
	constexpr uint32_t WinHeight = 900;

    sf::RenderWindow window(sf::VideoMode(WinWidth, WinHeight), "Hord", sf::Style::Default, settings);
    window.setVerticalSyncEnabled(true);

    Grid grid(20, 80, 45);

	DisplayManager display_manager(window);
    display_manager.event_manager.addKeyPressedCallback(sf::Keyboard::C, [&](const sf::Event& ev) {
        const sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);
        grid.setCellAt(sf::Vector2f(mouse_pos.x, mouse_pos.y), 1);
    });

    while (window.isOpen()) {
        display_manager.processEvents();

        window.clear(sf::Color::Black);

        Renderer::renderGrid(window, grid, display_manager.getRenderStates());

		window.display();
    }

    return 0;
}
