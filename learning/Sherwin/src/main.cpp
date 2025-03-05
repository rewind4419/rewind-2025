#include <stdio.h>
#include <functional>

#include <math.h>

class V2
{
public:
    float x;
    float y;

    V2(float x, float y) {this->x = x; this->y = y;}

    V2 operator+(V2& r)
    {
        return V2{this->x + r.x, this->y + r.y};
    }

    V2 operator-(V2& r)
    {
        return V2{this->x - r.x, this->y - r.y};
    }
    
    float dot(V2& r)
    {
        return this->x * r.x + this->y * r.y;
    }

    float cross(V2& r)
    {
        return this->x * r.y - this->y * r.x;
    }

    float length()
    {
        return sqrtf(this->x * this->x + this->y * this->y);
    }

    // Can be used instead of length if squared is fine, because it avoids the expensive sqrt
    float lengthSquared()
    {
        return this->x * this->x + this->y * this->y;
    }

    V2 normalize()
    {
        float len = this->length();
        return V2 {this->x / len, this->y / len};
    }
};

void print_hi()
{
    printf("Hello there!\n");
}

int main()
{
    // V2 a {2.0, 2.0};
    // V2 b {1.5, 1.5};

    // printf("A: (%f, %f)\n", a.x, a.y);
    // printf("B: (%f, %f)\n", b.x, b.y);

    // printf("Dot: %f\n", a.dot(b));
    // printf("Cross: %f\n", a.cross(b));

    // V2 normalized = a.normalize();

    // printf("A Length: %f\n", a.length());
    // printf("A LengthSquared: %f\n", a.lengthSquared());

    // printf("A Normalized: (%f, %f)\n", normalized.x, normalized.y);

    float baa = 0.5f;

    std::function<void()> my_function = [baa] {printf("Hello, my name is %f\n", baa);};

    my_function();

    baa = 1.0f;

    my_function();
}