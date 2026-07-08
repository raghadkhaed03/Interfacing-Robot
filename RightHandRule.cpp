#include <stdio.h>
#include "API_c.h"

void RightHandRule()
{
    while (1)
    {

        if (!API_wallRight())
        {
            API_turnRight();
        }

        while (API_wallFront())
        {
            API_turnLeft();
        }

        API_moveForward();
    }
}

int main(int argc, char *argv[])
{

    RightHandRule();

    return 0;
}