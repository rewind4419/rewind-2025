#include <stdio.h>

void doubleMyNumber(int* num){
    *num *=2;
}

void doubleMyNumber2(int& num){
    num *=2;
}

int main() {

    int a = 5;

    doubleMyNumber2(a);

    printf("%d\n", a);

}