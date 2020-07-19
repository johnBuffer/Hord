#pragma once
#include <vector>
#include <SFML/System/Vector2.hpp>
#include <sfml_tools.hpp>


struct HitPoint
{

};


struct Grid
{
public:
	Grid() = default;

	HitPoint castRayToPoint(const sf::Vector2f& start, const sf::Vector2f& end) const;

	HitPoint castRay(const sf::Vector2f& start, const sf::Vector2f& direction, const float max_dist) const
	{
		sf::Vector2f inv_direction(1.0f / direction.x, 1.0f / direction.y);
		sf::Vector2i cell_coords = toGridCoords(start);
		const sf::Vector2i step = Tools::vsign(direction);
		sf::Vector2f t_max((cell_coords.x - (step.x <= 0)) * cell_size, (cell_coords.y - (step.y <= 0)) * cell_size);
		t_max -= start;
		t_max.x *= inv_direction.x;
		t_max.y *= inv_direction.y;
		sf::Vector2f t_d(Tools::vabs(sf::Vector2f(cell_size * inv_direction.x, cell_size * inv_direction.y)));
		do {
			if (t_max.x < t_max.y) {
				t_max.x += t_d.x;
				cell_coords.x += step.x;
			}
			else {
				t_max.y += t_d.y;
				cell_coords.y += step.y;
			}
		} while (checkCondition());
	}

private:
	int32_t cell_size;

	sf::Vector2i toGridCoords(const sf::Vector2f& v) const
	{
		return sf::Vector2i(v.x / cell_size, v.y / cell_size);
	}

	bool checkCondition() const
	{

	}
};
