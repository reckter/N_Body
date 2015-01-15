#ifndef BODY_DEF
#define BODY_DEF

class Body {
private:
    float x;
    float y;

    float radius;

    float velocityX;
    float velocityY;

    float mass;

public:
    Body(float x, float y, float radius, float velocityX, float velocityY, float mass)
            : x(x), y(y), radius(radius), velocityX(velocityX), velocityY(velocityY), mass(mass) {
    }

    float getX() const {
        return x;
    }

    void setX(float x) {
        Body::x = x;
    }

    float getY() const {
        return y;
    }

    void setY(float y) {
        Body::y = y;
    }

    float getRadius() const {
        return radius;
    }

    void setRadius(float radius) {
        Body::radius = radius;
    }

    float getVelocityX() const {
        return velocityX;
    }

    void setVelocityX(float velocityX) {
        Body::velocityX = velocityX;
    }

    float getVelocityY() const {
        return velocityY;
    }

    void setVelocityY(float velocityY) {
        Body::velocityY = velocityY;
    }

    float getMass() const {
        return mass;
    }

    void setMass(float mass) {
        Body::mass = mass;
    }

    /**
    * chefks if the two bodys would collide
    */
    bool collidesWith(Body other) {
        float diffx = other.getX() - x;
        float diffy = other.getY() - y;
        float diff = diffx * diffx + diffy * diffy;
        return (other.getRadius() + radius) * (other.getRadius() + radius) >= diff;
    }
};

#endif