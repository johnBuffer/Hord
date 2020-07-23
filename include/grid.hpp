#pragma once
#include <vector>
#include <SFML/System/Vector2.hpp>
#include <sfml_tools.hpp>
#include <iostream>
#include "vec.hpp"


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

struct Cell
{
	uint8_t type;
	std::array<
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
		int32_t cell_coords[2];
		toGridCoords(start, cell_coords);
		const int32_t step[]{ Tools::sign(direction.x), Tools::sign(direction.y) };
		const float cell_size_f = Tools::as<float>(cell_size);
		const float inv_direction[]{ 1.0f / direction.x, 1.0f / direction.y };
		const float t_d[]{ std::abs(cell_size_f * inv_direction[0]), std::abs(cell_size_f * inv_direction[1]) };

		float t_max[]{
			((cell_coords[0] + (step[0] > 0)) * cell_size_f - start.x) * inv_direction[0],
			((cell_coords[1] + (step[1] > 0)) * cell_size_f - start.y) * inv_direction[1]
		};

		uint32_t min_index = (t_max[0] >= t_max[1]);
		while (t_max[min_index] < max_dist) {
			if (getCellContentAt(cell_coords[0], cell_coords[1]) == 1) {
				return HitPoint(true);
			}

			t_max[min_index] += t_d[min_index];
			cell_coords[min_index] += step[min_index];
			min_index = (t_max[0] >= t_max[1]);
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

	void toGridCoords(const sf::Vector2f& v, int32_t* out) const
	{
		out[0] = v.x / cell_size;
		out[1] = v.y / cell_size;
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
