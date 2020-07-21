#pragma once
#include <SFML/Graphics.hpp>
#include "grid.hpp"


struct Renderer
{
	static void renderGrid(sf::RenderTarget& target, const Grid& grid, const sf::RenderStates& rs)
	{
		const GridInfo grid_info = grid.getInfo();
		const int32_t cs = grid_info.cell_size;
		sf::VertexArray va(sf::Quads, 4 * grid_info.width * grid_info.height);
		for (int32_t x(0); x < grid_info.width; ++x) {
			for (int32_t y(0); y < grid_info.height; ++y) {
				const uint64_t index = x + y * grid_info.width;
				va[4 * index + 0].position = sf::Vector2f(x * cs, y * cs);
				va[4 * index + 1].position = sf::Vector2f((x+1) * cs, y * cs);
				va[4 * index + 2].position = sf::Vector2f((x+1) * cs, (y+1) * cs);
				va[4 * index + 3].position = sf::Vector2f(x * cs, (y+1) * cs);

				sf::Color color = grid.getCellContentAt(x, y) ? sf::Color::Black : sf::Color::White;
				va[4 * index + 0].color = color;
				va[4 * index + 1].color = color;
				va[4 * index + 2].color = color;
				va[4 * index + 3].color = color;
			}
		}

		target.draw(va, rs);
	}
};
