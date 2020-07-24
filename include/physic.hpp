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

	float dot(const Vec2& other) const
	{
		return x * other.x + y * other.y;
	}

	Vec2 getNormalized() const
	{
		return (*this) / getLength();
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
	Atom()
		: point()
		, frame_move()
		, mass(1.0f)
	{}

	Atom(const Vec2& p)
		: point(p.x, p.y)
		, frame_move()
		, mass(1.0f)
	{}

	float getDistance2With(const Atom& other) const
	{
		return point.getDistance2With(other.point);
	}

	void move(const Vec2& v)
	{
		frame_move += v;
	}

	void moveTo(const Vec2& p)
	{
		const Vec2 v = p - point.coords;
		move(v);
	}

	void update(float dt)
	{

	}

	void reset()
	{
		frame_move = {};
	}

	VerletPoint point;
	Vec2 frame_move;
	float mass;
	float radius = 12.0f;
};


struct ComposedObject
{
	ComposedObject()
		: center_of_mass()
		, speed()
		, angular_speed(10.0f)
		, angle(0.5f)
	{}

	void addAtom(const Vec2& p)
	{
		atoms.emplace_back(p);
	}

	void computeCenterOfMass()
	{
		Vec2 com;
		for (const Atom& a : atoms) {
			com += a.point.coords;
		}
		center_of_mass = com / float(atoms.size());
	}

	void solveBoundaryCollisions()
	{
		for (Atom& a : atoms) {
			Vec2 p = a.point.coords;
			if (a.point.coords.x > boundaries_max.x - a.radius) {
				p.x = boundaries_max.x - a.radius;
			}
			else if (a.point.coords.x < boundaries_min.x + a.radius) {
				p.x = boundaries_min.x + a.radius;
			}

			if (a.point.coords.y > boundaries_max.y - a.radius) {
				p.y = boundaries_max.y - a.radius;
			}
			else if (a.point.coords.y < boundaries_min.y + a.radius) {
				p.y = boundaries_min.y + a.radius;
			}
			a.moveTo(p);
		}
	}

	void update(float dt)
	{
		speed += Vec2(0.0f, 100.0f) * dt;
		translate(speed * dt);
		computeCenterOfMass();

		angle += angular_speed * dt;
		rotate(angular_speed * dt);

		solveBoundaryCollisions();
		Vec2 delta_p(0.0f, 0.0f);
		float delta_r = 0.0f;
		for (Atom& a : atoms) {
			delta_p += a.frame_move;
			delta_r += getRotationDelta(a);
			a.reset();
		}
		translate(delta_p);
		speed += delta_p * 10.0f;
	}

	float getRotationDelta(const Atom& a) const
	{
		const Vec2 to_com = center_of_mass - a.point.coords;
		// Could be pre computed
		const float length = to_com.getLength();
		if (length > 0.001f) {
			const Vec2 to_com_v = to_com / length;
		}


	}

	void translate(const Vec2& v)
	{
		for (Atom& a : atoms) {
			a.point.coords += v;
		}
	}

	void rotate(float r)
	{
		for (Atom& a : atoms) {
			a.point.coords.rotate(center_of_mass, r);
		}
	}

	std::vector<Atom> atoms;
	Vec2 center_of_mass;
	Vec2 speed;

	float angular_speed;
	float angle;

	const Vec2 boundaries_min = Vec2(50.0f, 50.0f);
	const Vec2 boundaries_max = Vec2(1550.0f, 850.0f);
};


struct Solver
{
	void solveBoundaryCollisions()
	{
		for (Atom& a : atoms) {
			Vec2 p = a.point.coords;
			if (a.point.coords.x > boundaries_max.x - a.radius) {
				p.x = boundaries_max.x - a.radius;
			}
			else if (a.point.coords.x < boundaries_min.x + a.radius) {
				p.x = boundaries_min.x + a.radius;
			}

			if (a.point.coords.y > boundaries_max.y - a.radius) {
				p.y = boundaries_max.y - a.radius;
			}
			else if (a.point.coords.y < boundaries_min.y + a.radius) {
				p.y = boundaries_min.y + a.radius;
			}
			a.moveTo(p);
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

		//solveCollisions();
		solveBoundaryCollisions();

		/*for (Atom& a : atoms) {
			a.point.update(dt);
		}*/
	}

	const Vec2 boundaries_min = Vec2(50.0f, 50.0f);
	const Vec2 boundaries_max = Vec2(1550.0f, 850.0f);
	std::vector<Atom> atoms;
};

