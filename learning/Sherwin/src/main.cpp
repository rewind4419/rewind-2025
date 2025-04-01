#include <stdio.h>

class MyClass
{
public:
    int number = 0;

};

int main()
{
    MyClass* name;

    name = new MyClass();

    delete name;

    name->number = 5;
}