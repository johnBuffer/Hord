#pragma once
#include <vector>
#include <SFML/System/Vector2.hpp>
#include <sfml_tools.hpp>
#include <iostream>


struct HitPoint
{
	HitPoint(bool hit_)
		: hit(hit_)
	{}

	bool hit;
};


struct GridInfo
{
	GridInfo(int32_t cell_size_, int32_t width_, int32_t height_)
		: cell_size(cell_size_)
		, width(width_)
		, height(height_)
	{}

	int32_t cell_size;
	int32_t width, height;
};


struct Grid
{
public:
	Grid(int32_t cell_size_, int32_t width_, int32_t height_)
		: cell_size(cell_size_)
		, width(width_)
		, height(height_)
		, data(Tools::as<uint64_t>(width) * Tools::as<uint64_t>(height), 0u)
	{}

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
		float distance = 0.0f;
		do {
			if (getCellContentAt(step.x, step.y)) {
				return HitPoint(true);
			}

			if (t_max.x < t_max.y) {
				t_max.x += t_d.x;
				distance += t_d.x;
				cell_coords.x += step.x;
			}
			else {
				t_max.y += t_d.y;
				distance += t_d.y;
				cell_coords.y += step.y;
			}
		} while (distance < max_dist);

		return HitPoint(false);
	}

	void setCellAt(const sf::Vector2f& world_position, uint8_t value)
	{
		std::cout << "OK" << std::endl;
		const sf::Vector2i grid_coords = toGridCoords(world_position);
		if (checkCoords(grid_coords.x, grid_coords.y)) {
			data[getIndexFromCoords(grid_coords.x, grid_coords.y)] = value;
		}
	}

	uint8_t getCellContentAt(int32_t x, int32_t y) const
	{
		if (checkCoords(x, y)) {
			return data[getIndexFromCoords(x, y)];
		}

		return 0u;
	}

	GridInfo getInfo() const
	{
		return GridInfo(cell_size, width, height);
	}

private:
	int32_t cell_size;
	int32_t width;
	int32_t height;
	std::vector<uint8_t> data;

private:
	sf::Vector2i toGridCoords(const sf::Vector2f& v) const
	{
		return sf::Vector2i(v.x / cell_size, v.y / cell_size);
	}

	bool checkCondition() const
	{

	}

	uint64_t getIndexFromCoords(int32_t x, int32_t y) const
	{
		return Tools::as<uint64_t>(x) + Tools::as<uint64_t>(y) * Tools::as<uint64_t>(width);
	}

	bool checkCoords(int32_t x, int32_t y) const
	{
		return (x >= 0 && y >= 0 && x < width && y < height);
	}
};
