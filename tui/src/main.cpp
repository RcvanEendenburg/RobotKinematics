#include <tui/Application.h>

int main(int argc, char **argv)
{
  Application app(argc, argv, argv[1]);
  app.run();
  return 0;
}
