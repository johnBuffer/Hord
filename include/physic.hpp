#pragma once
#include <vector>


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

	Vec2 operator/(float f) const
	{
		const float inv = 1.0f / f;
		return Vec2(x * inv, y * inv);
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

	void operator-=(const Vec2& other)
	{
		x -= other.x;
		y -= other.y;
	}

	float x, y;
};


struct VerletPoint
{
	VerletPoint()
		: coords()
		, last_coords()
		, acceleration()
	{}

	VerletPoint(float x, float y)
		: coords(x, y)
		, last_coords(x, y)
		, acceleration()
	{}

	void update(float dt)
	{
		const float new_x = coords.x + (coords.x - last_coords.x) * dt + acceleration.x * dt * dt;
		const float new_y = coords.y + (coords.y - last_coords.y) * dt + acceleration.y * dt * dt;
		last_coords = coords;
		coords.x = new_x;
		coords.y = new_y;
		acceleration = {};
	}

	float getDistance2With(const VerletPoint& other) const
	{
		const float dx = coords.x - other.coords.x;
		const float dy = coords.y - other.coords.y;
		return dx*dx + dy*dy;
	}

	float getDistanceWith(const VerletPoint& other) const
	{
		return sqrt(getDistance2With(other));
	}

	void move(const Vec2& v)
	{
		coords += v;
	}

	void accelerate(const Vec2& a)
	{
		acceleration += a;
	}

	Vec2 coords, last_coords;
	Vec2 acceleration;
};


struct Atom
{
	VerletPoint point;
	float mass;

	float radius = 12.0f;

	float getDistance2With(const Atom& other) const
	{
		return point.getDistance2With(other.point);
	}
};


struct Solver
{

	void solveBoundaryCollisions()
	{
		for (Atom& a : atoms) {
			if (a.point.coords.x > boundaries.x - a.radius) {
				a.point.coords.x = boundaries.x - a.radius;
			}

			if (a.point.coords.x < a.radius) {
				a.point.coords.x = a.radius;
			}

			if (a.point.coords.y > boundaries.y - a.radius) {
				a.point.coords.y = boundaries.y - a.radius;
			}
		}
	}

	void solveCollisions()
	{
		const uint64_t atoms_count = atoms.size();
		for (uint64_t i(0); i++;) {
			Atom& a1 = atoms[i];
			for (uint64_t k(i+1); k++;) {
				Atom& a2 = atoms[k];
				const Vec2 v = a1.point.coords - a2.point.coords;
				const float dist2 = v.getLength2();
				const float min_dist = 2.0f * a1.radius;

				if (dist2 < min_dist * min_dist) {
					const float dist = sqrt(dist2);
					const float delta_dist = min_dist - dist;
					const Vec2 axis = v / dist;

					const Vec2 positions_correction(delta_dist * axis.x, delta_dist * axis.y);
					a1.point.move( positions_correction);
					a2.point.move(-positions_correction);
				}
			}
		}
	}

	void update(float dt)
	{
		const Vec2 gravity(0.0f, 10.0f);
		for (Atom& a : atoms) {
			a.point.accelerate(gravity);
		}

		solveCollisions();
		solveBoundaryCollisions();

		for (Atom& a : atoms) {
			a.point.update(dt);
		}
	}

	const Vec2 boundaries = Vec2(1600.0f, 900.0f);
	std::vector<Atom> atoms;
};
