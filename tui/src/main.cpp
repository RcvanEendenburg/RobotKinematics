#include <tui/Application.h>

int main(int argc, char **argv)
{
  Application app(argc, argv, "/home/derk/workspace/src/tui/config/config.ini");
  app.run();
  return 0;
}