#pragma once
#include <cstdarg>

template<typename T, unsigned int N>
struct Vec
{
	Vec(T args...)
		: coords{args}
	{}

	T coords[N];
};


//template<typename T, unsigned int N>
//using Vec = T[N];
//
//using Vec2f = Vec<float, 2>;

template<typename T>
struct Vec2 : public Vec<T, 2>
{
	Vec2()
		: Vec<T, 2>(0, 0)
		, x(coords[0])
		, y(coords[1])
	{}

	Vec2(T x_, T y_)
		: Vec<T, 2>(x_, y_)
		, x(coords[0])
		, y(coords[1])
	{}

	T& x;
	T& y;
};


using Vec2f = Vec2<float>;
using Vec2i = Vec2<int32_t>;

template<typename T>
Vec2<T> vabs(const Vec2<T>& v)
{
	return Vec2<T>(std::abs(v.coords[0]), std::abs(v.coords[1]));
}

template<typename T>
static T sign(T x)
{
	return (x >= as<int, T>(0)) + as<int, T>(-1) * (x < as<int, T>(0));
}

template<typename T>
Vec2i vsign(const Vec2<T>& v)
{
	return Vec2i(sign(v.coords[0]), sign(v.coords[1]));
}
