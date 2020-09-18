#pragma once
#include <vector>
#include <SFML/System/Vector2.hpp>
#include <sfml_tools.hpp>
#include <iostream>
#include <array>
#include "vec.hpp"
#include "physic_objects.hpp"


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


struct CellObject
{
	Atom* atom;
	uint64_t atom_id;
};


struct Cell
{
	Cell()
		: count(0)
	{}

	uint64_t count;
	uint8_t type;
	std::array<CellObject, 16> objects;

	void addObject(Atom& a, uint64_t id)
	{
		if (count >= 16) {
			std::cout << "Cell overflow" << std::endl;
			return;
		}

		objects[count] = {
			&a,
			id
		};
		++count;
	}

	void reset()
	{
		count = 0;
	}
};


struct Grid
{
public:
	struct Config {
		static const uint64_t safety_margin = 5;
	};

	Grid(int32_t cell_size_, int32_t width_, int32_t height_)
		: cell_size(cell_size_)
		, width(width_ + 2 * Config::safety_margin)
		, height(height_ + 2 * Config::safety_margin)
		, data(Tools::as<uint64_t>(width) * Tools::as<uint64_t>(height))
	{}

	HitPoint castRayToPoint(const sf::Vector2f& start, const sf::Vector2f& end)
	{
		return castRay(start, Tools::normalize(end - start), Tools::length(end - start));
	}

	HitPoint castRay(const sf::Vector2f& start, const sf::Vector2f& direction, const float max_dist)
	{
		int32_t cell_coords[2];
		toGridCoords(start, cell_coords);
		const int32_t step[]{ Tools::as<int32_t>(Tools::sign(direction.x)), Tools::as<int32_t>(Tools::sign(direction.y)) };
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

	void clear()
	{
		for (Cell& c : data) {
			c.reset();
		}
	}

	uint8_t getCellContentAtWorld(const sf::Vector2f& world_position)
	{
		const sf::Vector2i grid_coords = toGridCoords(world_position);
		return getCellContentAt(grid_coords.x, grid_coords.y);
	}

	uint64_t getCellContentAt(int32_t x, int32_t y) const
	{
		if (checkCoords(x, y)) {
			return data[getIndexFromCoords(x, y)].count;
		}

		return 0u;
	}

	GridInfo getInfo() const
	{
		return GridInfo(cell_size, width, height);
	}

	sf::Vector2f getDiscretizedCoords(const sf::Vector2f& world_coords) const
	{
		const sf::Vector2i grid_coords = toGridCoords(world_coords);
		return float(cell_size) * (sf::Vector2f(grid_coords.x, grid_coords.y) + sf::Vector2f(0.5f, 0.5f));
	}

	void addObject(Atom& a, uint64_t id)
	{
		const sf::Vector2i grid_coords = toGridCoords(a.position);
		if (!checkCoords(grid_coords.x, grid_coords.y)) {
			//std::cout << "Skip" << std::endl;
			return;
		}

		const uint64_t grid_index = getIndexFromCoords(grid_coords.x, grid_coords.y);
		// Add at current position
		addToCell(grid_index, a, id);
		a.grid_index = grid_index;
		// Add left and right
		addToCell(grid_index - 1, a, id);
		addToCell(grid_index + 1, a, id);
		// Add Top
		addToCell(grid_index - width    , a, id);
		addToCell(grid_index - width - 1, a, id);
		addToCell(grid_index - width + 1, a, id);
		// Add bottom
		addToCell(grid_index + width    , a, id);
		addToCell(grid_index + width - 1, a, id);
		addToCell(grid_index + width + 1, a, id);
	}

	void addToCell(uint64_t cell_id, Atom& a, uint64_t id)
	{
		data[cell_id].addObject(a, id);
	}

	Cell& getCell(uint64_t id)
	{
		return data[id];
	}

private:
	const int32_t cell_size;
	const int32_t width;
	const int32_t height;
	mutable std::vector<Cell> data;

private:
	template<typename T>
	sf::Vector2i toGridCoords(const T& v) const
	{
		return sf::Vector2i(Tools::as<int32_t>(v.x / cell_size + Config::safety_margin), Tools::as<int32_t>(v.y / cell_size + Config::safety_margin));
	}

	void toGridCoords(const sf::Vector2f& v, int32_t* out) const
	{
		out[0] = Tools::as<int32_t>(v.x / cell_size);
		out[1] = Tools::as<int32_t>(v.y / cell_size);
	}

	uint64_t getIndexFromCoords(int32_t x, int32_t y) const
	{
		return Tools::as<uint64_t>(x) + Tools::as<uint64_t>(y) * Tools::as<uint64_t>(width);
	}

	bool checkCoords(int32_t x, int32_t y) const
	{
		return (x >= Config::safety_margin-1 && y >= Config::safety_margin-1 && x < width - Config::safety_margin && y < height - Config::safety_margin);
	}
};
