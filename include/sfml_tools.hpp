#pragma once
#include <SFML/System/Vector2.hpp>
#include <cmath>


struct Tools
{
	static float length(const sf::Vector2f& v)
	{
		return sqrt(v.x * v.x + v.y * v.y);
	}

	static sf::Vector2f normalize(const sf::Vector2f& v)
	{
		const float inv_length = 1.0f / length(v);
		return sf::Vector2f(v.x * inv_length, v.y * inv_length);
	}

	template<typename U, typename T>
	static U as(const T& obj)
	{
		return static_cast<U>(obj);
	}

	template<typename T>
	static T sign(T x)
	{
		return (x >= as<int, T>(0)) + as<int, T>(-1) * (x < as<int, T>(0));
	}

	template<typename T>
	static sf::Vector2i vsign(const sf::Vector2<T>& v)
	{
		const T zero = 0;
		return sf::Vector2i(as<T>(sign(v.x)), as<T>(sign(v.y)));
	}

	template<typename T>
	static sf::Vector2<T> vabs(const sf::Vector2<T>& v)
	{
		return sf::Vector2<T>(std::abs(v.x), std::abs(v.y));
	}

	template<typename T>
	static const T* asArray(sf::Vector2<T>& v)
	{
		return (const T*)&v;
	}
};
