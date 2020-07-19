#include <SFML/Graphics.hpp>
#include <vector>
#include <list>
#include <cmath>
#include <iostream>
#include <fstream>

#include "display_manager.hpp"


int main()
{
	sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
	constexpr uint32_t WinWidth  = 1600;
	constexpr uint32_t WinHeight = 900;

    sf::RenderWindow window(sf::VideoMode(WinWidth, WinHeight), "Hord", sf::Style::Default, settings);
    window.setVerticalSyncEnabled(true);

	DisplayManager display_manager(window);
 
    while (window.isOpen()) {
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);

        window.clear(sf::Color::Black);

		window.display();
    }

    return 0;
}
