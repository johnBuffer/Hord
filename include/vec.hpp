#pragma once


struct Vec2
{
	Vec2()
		: x(0.0f)
		, y(0.0f)
	{}

	Vec2(float x_, float y_)
		: x(x_)
		, y(y_)
	{}

	float getLength2() const
	{
		return x * x + y * y;
	}

	float getLength() const
	{
		return sqrt(getLength2());
	}

	Vec2 operator/(float f) const
	{
		const float inv = 1.0f / f;
		return Vec2(x * inv, y * inv);
	}

	Vec2 operator*(float f) const
	{
		return Vec2(x * f, y * f);
	}

	Vec2 operator-(const Vec2& other) const
	{
		return Vec2(x - other.x, y - other.y);
	}

	Vec2 operator-() const
	{
		return Vec2(-x, -y);
	}

	void operator+=(const Vec2& other)
	{
		x += other.x;
		y += other.y;
	}

	Vec2 plus(const Vec2& other) const
	{
		return Vec2(x + other.x, y + other.y);
	}

	void operator-=(const Vec2& other)
	{
		x -= other.x;
		y -= other.y;
	}

	void rotate(const Vec2& origin, float angle)
	{
		const Vec2 v = *this - origin;

		// This should be precomputed
		const float ca = cos(angle);
		const float sa = sin(angle);

		const float new_x = v.x * ca - v.y * sa;
		const float new_y = v.x * sa + v.y * ca;

		x = new_x + origin.x;
		y = new_y + origin.y;
	}

	Vec2 getNormal() const
	{
		return Vec2(-y, x);
	}

	float dot(const Vec2& other) const
	{
		return x * other.x + y * other.y;
	}

	float cross(const Vec2& other) const
	{
		return x * other.y - y * other.x;
	}

	Vec2 getNormalized() const
	{
		return (*this) / getLength();
	}

	float x, y;
};
