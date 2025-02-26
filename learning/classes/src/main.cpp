#include <stdio.h>
#include "header.h"
#include <iostream>
#include "header2.h"
using namespace std;

struct elevator
{
    float current_height;
};
struct part
{
    float random_float;
};
// struct robot
// {
//     int width;
//     int height;
//     part robot_part;
//     elevator my_elevator;
// };


class robot
{
public:
    int width;
    int height;
    part robot_part;
    elevator my_elevator;
    void my_function(){
        
        printf("number: %d\n", width*height);
    }
};


int main() {
    robot my_robot {
        5,
        4,
        part{10.3},
        elevator{10.4}
    };
    robot my_robot2 {
        .width=4,
        .height=3,
        .robot_part=part{.random_float=10.5},
        .my_elevator=elevator{.current_height=10.4}
    };
    my_robot.my_function();
    my_robot2.my_function();

    amazingclass awesomeclass;
    awesomeclass.amazingint = 7;
    printf("%d\n", awesomeclass.amazingfunc());
    // float my_variable = 5.0f;
    lessamazingfunc();

    // robot my_robot;
    // robot my_robot_2;

    // my_robot.width = 2;
    // my_robot.robot_part.random_float = 10.3;
    // my_robot_2.width = 3;
    
    // printf("My robot has a width of %d and a height of %d\n", my_robot.width, my_robot.height);
}