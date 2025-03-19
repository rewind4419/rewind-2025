#include <cmath>
#include <cstdio>

class Complex
{
    public:
        double real;
        double imag;

        Complex(double r=0.0, double i=0.0)
        {
            real = r;
            imag = i;
        }
        Complex operator+(Complex pIn)
        {
            return Complex(real+pIn.real, imag + pIn.imag);
        }
        Complex operator-(Complex pIn)
        {
            return Complex(real-pIn.real, imag - pIn.imag);
        }
        Complex operator*(Complex pIn)
        {
            return Complex(real*pIn.real - imag*pIn.imag, real * pIn.imag + imag*pIn.real);
        }

        Complex pow(double exponent)
        {
            double mag = std::pow(sqrt(real*real + imag*imag),exponent);
            double theta = atan2(imag,real) * exponent;

            return Complex(cos(theta)*mag,sin(theta)*mag);
        }
        Complex inverse()
        {
            double mag = real*real + imag*imag;

            return Complex(real / mag, -imag/mag);
        }
        void print()
        {
            if(imag >= 0.0){
                printf("%lf + %lfi\n",real,imag);
            }
            else {
                printf("%lf - %lfi\n",real,-imag);
            }
        }
};
Complex operator*(Complex comp, double value)
{
    return Complex(comp.real * value, comp.imag * value);
}

Complex operator*(double value, Complex comp)
{
    return Complex(comp.real * value, comp.imag * value);
}

Complex operator+(Complex comp, double value)
{
    return Complex(comp.real + value, comp.imag);
}
Complex operator+(double value,Complex comp)
{
    return Complex(comp.real + value, comp.imag);
}

Complex operator-(Complex comp, double value)
{
    return Complex(comp.real - value, comp.imag);
}
Complex operator-(double value,Complex comp)
{
    return Complex(-comp.real + value, -comp.imag);
}

class Vec3
{
    public:
        double x;
        double y;
        double r;

        Vec3(double xInit = 0.0,double yInit = 0.0, double rInit = 0.0)
        {
            x = xInit;
            y = yInit;
            r = rInit;
        }
    
        Vec3 operator+(Vec3 pIn)
        {
            return Vec3(x+pIn.x,y+pIn.y,r+pIn.r);
        }

        Vec3 operator-(Vec3 pIn)
        {
            return Vec3(x-pIn.x,y-pIn.y,r-pIn.r);
        }
        double dot(Vec3 pIn)
        {
            return x * pIn.x + y*pIn.y + r*pIn.r;
        }
        double mag()
        {
            return  sqrt(x*x+y*y+r*r);
        }
        void print()
        {
            printf("x: %lf, y: %lf, r: %lf\n",x,y,r);
        }
        void scale(double scalar)
        {
            x *= scalar;
            y *= scalar; 
            r *= scalar;
        }
        Vec3 normalized()
        {
            Vec3 out = Vec3(x,y,r);
            double mag = out.mag();
            if(out.mag() != 0.0)
            {
                out.scale(1.0/mag);
            }
            return  out;
        }
};
Vec3 operator*(Vec3 vec, double value)
{
    return Vec3(vec.x*value, vec.y*value, vec.r*value);
}
Vec3 operator*(double value,Vec3 vec)
{
    return Vec3(vec.x*value, vec.y*value, vec.r*value);
}

class BezierPath
{
    public:
        Vec3 p0;
        Vec3 p1;
        Vec3 p2;
        BezierPath(Vec3 start, Vec3 ctrl, Vec3 end)
        {
            p0 = start;
            p1 = ctrl;
            p2 = end;
        }

        Vec3 getPoint(double t)
        {
            return (p2- 2*p1 + p0)*t*t + 2*(p1-p0)*t + p0;
        }

        Vec3 getVelocity(double t)
        {
            return 2*(p2- 2*p1 + p0)*t + 2*(p1-p0);
        }
        double getBezierClosestPoint(Vec3 point)
        {
            Vec3 q = p0 + p2 - 2*p1;
            Vec3 p = p1 - p0;
        
            double a = q.dot(q);
            double b = 3 * p.dot(q);
            double c = 2*p.dot(p) + (p0-point).dot(q);
            double d = (p0-point).dot(p);
        
            double d0 = b*b - 3*a*c;
            double d1 = 2*b*b*b - 9*a*b*c + 27*a*a*d;
        
            Complex rootOfUnity = Complex(-0.5,sqrt(3.0)*0.5);
        
            Complex C [3];
        
            C[0] = (0.5*(Complex(d1,0.0) + Complex(d1*d1-4.0*d0*d0*d0,0.0).pow(0.5))).pow(1.0/3.0);
        
            C[1] = C[0] * rootOfUnity;
            C[2] = C[1] * rootOfUnity;
        
            double minDist = INFINITY;
        
            double out = 0.0;
        
            for(int i = 0; i < 3; i++)
            {
                Complex t = (-1.0 / (3.0*a)) * (b + C[i] + d0 * C[i].inverse());
                if(fabs(t.imag) < 0.0000000001)
                {
                    Vec3 closestPointCandidate = getPoint(t.real);
                    double dist = (closestPointCandidate - point).mag();
        
                    if(dist < minDist)
                    {
                        minDist = dist;
                        out = t.real;
                    }
                }
            }
        
            return out;
        }

        double speed(double t)
        {
            return sqrt(4-4*t);
        }

        Vec3 getRobotControlVelocity(Vec3 robotPosition, Vec3 robotVelocity, double dT)
        {
            double closestTime = fmin(getBezierClosestPoint(robotPosition),1.0);

            double lookaheadTime  = closestTime + 0.1;


            Vec3 lookaheadPoint = getPoint(lookaheadTime);
            Vec3 lookaheadVelocity = getVelocity(lookaheadTime);

            Vec3 robotPathV = robotVelocity * (getVelocity(closestTime).mag() / (speed(closestTime) + 1.0));
            robotPathV.print();

            double accelComp = fmax(
                0.5 * (
                    (lookaheadPoint - robotPosition).dot(lookaheadVelocity + robotPathV)/
                    (lookaheadPoint - robotPosition).dot(lookaheadPoint - robotPosition)
                ),
                1.0
            );


            Vec3 correctionAccel = 2.0 * (
                3.0 * lookaheadPoint - 3.0 * robotPosition - (1/accelComp)*(lookaheadVelocity + 2.0 * robotPathV)
            ) * accelComp * (robotVelocity.mag() / robotPathV.mag());

            printf("am: %lf\n",accelComp);
            correctionAccel.print();

            Vec3 closestDirection = getVelocity(closestTime).normalized();

            Vec3 forwardAccel = (1.0 / dT) * ( speed(closestTime) - (closestDirection.dot(robotVelocity + correctionAccel * dT))) * closestDirection;

            Vec3 robotAccel = (correctionAccel + forwardAccel) * (1.0 / fmax(
                0.25 * (correctionAccel + forwardAccel).mag(), 1.0
            ));

            Vec3 newVelocity = robotVelocity + robotAccel * dT;

            return newVelocity;
        }
};



int main()
{
    BezierPath curve = BezierPath(
        Vec3(-3.9,-4.8,0.0),
        Vec3(-2,4.4,0.0),
        Vec3(6.34,2.9,0.0)
    );
    Vec3 robotPos = Vec3(3.0, 1.0, 0.0);
    Vec3 robotVelocity = Vec3(0.98153,0.19,0.0);

    Vec3 updatedV = curve.getRobotControlVelocity(robotPos, robotVelocity, 0.05);
    updatedV.print();

    // printf("t0: %lf\n",t0);
    return 0;
}