#include <al5d/Application.h>
#include <iostream>

int
main(int argc, char **argv)
{
    try
    {
        Application application(argc, argv, "/home/derk/workspace/src/al5d/config/config.ini");
        application.run();
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}