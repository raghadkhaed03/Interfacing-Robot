
#include <stdio.h>
#include "API_c.h"

void LeftHandRule()
{
    while (1)
    {

        if (!API_wallLeft())
        {
            API_turnLeft();
        }

        while (API_wallFront())
        {
            API_turnRight();
        }

        API_moveForward();
    }
}

int main(int argc, char *argv[])
{

    LeftHandRule();

    return 0;
}