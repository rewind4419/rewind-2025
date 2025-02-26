#include <stdio.h>
#include "inherit.h"

void printathing(fancyschmancyclass& fsc)
{
    fsc.justafunction();
}

int main() {
    fancyschmancyclass fsc;

    printathing(fsc);



    fancierschmancierclassier fsc2;

    printathing(fsc2);
}