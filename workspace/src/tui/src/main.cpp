#include <tui/Application.h>

int main(int argc, char **argv)
{
    if(argc > 1) {
        Application app(argc, argv, argv[1]);
        app.run();
    }
  return 0;
}
