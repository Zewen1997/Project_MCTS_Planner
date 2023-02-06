#ifndef MCTS_GEOMETRY_H
#define MCTS_GEOMETRY_H

struct Vector2
{
  float x;
  float y;
  Vector2(){}
  Vector2(float const in_x, float const in_y):x(in_x), y(in_y){}
  Vector2 operator- (Vector2 const& obj) const
{
	Vector2 vec;
	vec.x = x - obj.x;
	vec.y = y - obj.y;

	return vec;
}

  Vector2 operator+ (Vector2 const& obj) const
{
	Vector2 vec;
	vec.x = x + obj.x;
	vec.y = y + obj.y;

	return vec;
}

  Vector2 operator* (float const& w)
{
	Vector2 vec;
	vec.x = x * w;
    vec.y = y * w;
	return vec;
}
    
  Vector2 operator= (Vector2 const& obj)
{
	Vector2 vec;
	x = obj.x;
    y = obj.y;
	return vec;
}
  Vector2 operator/ (float const& w)
{
	Vector2 vec;
	vec.x = x / w;
    vec.y = y / w;
	return vec;
}
  float squaredLength()
{
    return x * x + y * y; 
}
};

float dot(const Vector2& a, const Vector2& b) 
{
    return a.x * b.x + a.y * b.y; 
}

class OBB2D {
private:
    /** Corners of the box, where 0 is the lower left. */
    Vector2         corner[4];

    /** Two edges of the box extended away from corner[0]. */
    Vector2         axis[2];

    /** origin[a] = dot(corner[0], axis[a]); */
    double          origin[2];

    /** Returns true if other overlaps one dimension of this. */
    bool overlaps1Way(const OBB2D& other) const {
        for (int a = 0; a < 2; ++a) {

            double t = dot(other.corner[0], axis[a]);

            // Find the extent of box 2 on axis a
            double tMin = t;
            double tMax = t;

            for (int c = 1; c < 4; ++c) {
                t = dot(other.corner[c], axis[a]);

                if (t < tMin) {
                    tMin = t;
                } else if (t > tMax) {
                    tMax = t;
                }
            }

            // We have to subtract off the origin

            // See if [tMin, tMax] intersects [0, 1]
            if ((tMin > 1 + origin[a]) || (tMax < origin[a])) {
                // There was no intersection along this dimension;
                // the boxes cannot possibly overlap.
                return false;
            }
        }

        // There was no dimension along which there is no intersection.
        // Therefore the boxes overlap.
        return true;
    }

 
    /** Updates the axes after the corners move.  Assumes the
        corners actually form a rectangle. */
    void computeAxes() {
        axis[0] = corner[1] - corner[0]; 
        axis[1] = corner[3] - corner[0]; 

        // Make the length of each axis 1/edge length so we know any
        // dot product must be less than 1 to fall within the edge.

        for (int a = 0; a < 2; ++a) {
            axis[a] = axis[a] / axis[a].squaredLength();
            origin[a] = dot(corner[0], axis[a]);
        }
    }

public:

    OBB2D(const Vector2& center, const float w, const float h, float angle)
{
        Vector2 X( cos(angle), sin(angle));
        Vector2 Y(-sin(angle), cos(angle));
        X = X * w / 2.0f;
        Y = Y * h / 2.0f;

        corner[0] = center - X - Y;
        corner[1] = center + X - Y;
        corner[2] = center + X + Y;
        corner[3] = center - X + Y;

        computeAxes();
    }


    /** For testing purposes. */
    void moveTo(const Vector2& center) {
        Vector2 centroid = (corner[0] + corner[1] + corner[2] + corner[3]) / 4;

        Vector2 translation = center - centroid;

        for (int c = 0; c < 4; ++c) {
            corner[c] = corner[c] + translation;
        }

        computeAxes();
    }

    /** Returns true if the intersection of the boxes is non-empty. */
    bool overlaps(const OBB2D& other) const {
        return overlaps1Way(other) && other.overlaps1Way(*this);
    }
};

#endif
