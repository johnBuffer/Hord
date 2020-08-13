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

	Vec2 getNormal() const
	{
		return Vec2(-y, x);
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

struct Atom
{
	Atom()
		: position()
		, delta_position()
		, acceleration()
		, mass(1.0f)
	{}

	Atom(const Vec2& p)
		: position(p.x, p.y)
		, delta_position()
		, acceleration()
		, mass(1.0f)
	{}

	float getDistance2With(const Atom& other) const
	{
		return (position - other.position).getLength();
	}

	void move(const Vec2& v)
	{
		delta_position += v;
	}

	void moveTo(const Vec2& p)
	{
		const Vec2 v = p - position;
		move(v);
	}

	void update(float dt)
	{

	}

	void reset()
	{
		delta_position = {};
		acceleration = {};
	}

	Vec2 position;
	Vec2 delta_position;
	Vec2 acceleration;
	float mass;
	float radius = 12.0f;
};


struct ComposedObject
{
	ComposedObject()
		: center_of_mass()
		, speed()
		, angular_speed(0.0f)
		, angle(0.0f)
		, applied_force(0.0f, 0.0f)
	{}

	void addAtom(const Vec2& p)
	{
		atoms.emplace_back(p);
	}

	void computeCenterOfMass()
	{
		Vec2 com;
		for (const Atom& a : atoms) {
			com += a.position;
		}
		center_of_mass = com / float(atoms.size());
	}

	void applyForce(const Vec2& f)
	{
		applied_force += f;
	}

	void accelerate(const Vec2& a)
	{
		applyForce(a * getMass());
	}

	float getMass() const
	{
		return float(atoms.size());
	}

	void update(float dt)
	{
		// Stuff
		const float mass = 1.0f / getMass();
		speed += (applied_force / mass) * dt;
		// Reset
		applied_force = Vec2(0.0f, 0.0f);
	}

	void updateState(float dt)
	{
		translate(speed * dt);
		center_of_mass += speed * dt;

		rotate(angular_speed * dt);
		angle += angular_speed * dt;
	}

	void solveCollisions()
	{
		
	}

	void translate(const Vec2& v)
	{
		for (Atom& a : atoms) {
			a.position += v;
		}
	}

	void rotate(float r)
	{
		for (Atom& a : atoms) {
			a.position.rotate(center_of_mass, r);
		}
	}

	float getDistanceToCenterOfMass(const Vec2& p) const
	{
		return (center_of_mass - p).getLength();
	}

	std::vector<Atom> atoms;
	Vec2 center_of_mass;
	Vec2 speed;
	Vec2 applied_force;

	float angular_speed;
	float angle;
};


struct Solver
{
	void solveCollisions()
	{
		
	}

	void solveBoundaryCollisions()
	{
		for (ComposedObject& o : objects) {
			for (Atom& a : o.atoms) {
				Vec2 p = a.position;
				Vec2 contact_point;
				Vec2 normal;
				
				if (a.position.x > boundaries_max.x - a.radius) {
					p.x = boundaries_max.x - a.radius;
				}
				else if (a.position.x < boundaries_min.x + a.radius) {
					p.x = boundaries_min.x + a.radius;
				}

				if (a.position.y > boundaries_max.y - a.radius) {
					p.y = boundaries_max.y - a.radius;
				}
				else if (a.position.y < boundaries_min.y + a.radius) {
					p.y = boundaries_min.y + a.radius;
				}
				a.moveTo(p);
			}
		}
	}

	void applyGravity()
	{
		const Vec2 gravity(0.0f, 10.0f);
		for (ComposedObject& o : objects) {
			o.accelerate(gravity);
		}
	}

	void update(float dt)
	{
		applyGravity();

		for (ComposedObject& o : objects) {
			o.update(dt);
		}

		solveBoundaryCollisions();

		for (ComposedObject& o : objects) {
			o.updateState(dt);
		}

		/*solveCollisions();

		for (ComposedObject& o : objects) {
			o.solveCollisions();
		}*/
	}

	std::vector<ComposedObject> objects;
	const Vec2 boundaries_min = Vec2(50.0f, 50.0f);
	const Vec2 boundaries_max = Vec2(1550.0f, 850.0f);
};
