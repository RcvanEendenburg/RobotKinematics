#include <al5d/Application.h>
#include <iostream>

int
main(int argc, char **argv)
{
    try
    {
        if(argc > 1)
        {
            Application application(argc, argv, argv[1]);
            application.run();
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}