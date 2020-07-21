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

	HitPoint castRayToPoint(const sf::Vector2f& start, const sf::Vector2f& end)
	{
		return castRay(start, Tools::normalize(end - start), Tools::length(end - start));
	}

	HitPoint castRay(const sf::Vector2f& start, const sf::Vector2f& direction, const float max_dist)
	{
		clearDebug();

		const float cell_size_f = Tools::as<float>(cell_size);
		const sf::Vector2f inv_direction(1.0f / direction.x, 1.0f / direction.y);
		sf::Vector2i cell_coords = toGridCoords(start);
		const sf::Vector2i step = Tools::vsign(direction);
		sf::Vector2f t_max;

		sf::Vector2f t_d(Tools::vabs(sf::Vector2f(cell_size_f * inv_direction.x, cell_size_f * inv_direction.y)));
		
		t_max.x = ((cell_coords.x + (step.x > 0)) * cell_size_f - start.x) * inv_direction.x;
		t_max.y = ((cell_coords.y + (step.y > 0)) * cell_size_f - start.y) * inv_direction.y;
		
		while (std::min(t_max.x, t_max.y) < max_dist) {
			if (getCellContentAt(cell_coords.x, cell_coords.y) == 1) {
				return HitPoint(true);
			}

			setCellAt(cell_coords.x, cell_coords.y, 2);

			if (t_max.x < t_max.y) {
				t_max.x += t_d.x;
				cell_coords.x += step.x;
			}
			else {
				t_max.y += t_d.y;
				cell_coords.y += step.y;
			}
		}

		return HitPoint(false);
	}

	void setCellAtWorld(const sf::Vector2f& world_position, uint8_t value)
	{
		const sf::Vector2i grid_coords = toGridCoords(world_position);
		setCellAt(grid_coords.x, grid_coords.y, value);
	}

	void setCellAt(int32_t x, int32_t y, uint8_t value)
	{
		if (checkCoords(x, y)) {
			data[getIndexFromCoords(x, y)] = value;
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
	mutable std::vector<uint8_t> data;

private:
	sf::Vector2i toGridCoords(const sf::Vector2f& v) const
	{
		return sf::Vector2i(v.x / cell_size, v.y / cell_size);
	}

	void clearDebug()
	{
		for (uint8_t& c : data) {
			if (c == 2) {
				c = 0;
			}
		}
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
