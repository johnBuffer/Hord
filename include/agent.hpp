#pragma once
#include <SFML/Graphics.hpp>
#include "grid.hpp"
#include "sfml_tools.hpp"


struct Agent
{
	Agent() = default;

	Agent(float x, float y)
		: position(x, y)
		, direction(0.0f, 0.0f)
		, speed(60.0f)
		, last_target_position(position)
	{

	}

	void update(sf::Vector2f& target, Grid& grid, float dt)
	{
		if (!grid.castRayToPoint(position, target).hit) {
			last_target_position = target;

			direction = Tools::normalize(target - position);
		}
		else {
			direction = Tools::normalize(last_target_position - position);
		}

		position += (speed * dt) * direction;
	}

	void draw(sf::RenderTarget& target, const sf::RenderStates& rs) const
	{
		const float radius = 12.0f;
		sf::CircleShape c(radius);
		c.setOrigin(radius, radius);
		c.setPosition(position);
		c.setFillColor(sf::Color::Red);

		target.draw(c, rs);
	}

	sf::Vector2f position;
	sf::Vector2f direction;
	sf::Vector2f last_target_position;

	float speed;
};


