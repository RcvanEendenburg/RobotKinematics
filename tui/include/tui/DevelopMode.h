//
// Created by derk on 27-4-20.
//

#ifndef TUI_INCLUDE_TUI_DEVELOPMODE_H
#define TUI_INCLUDE_TUI_DEVELOPMODE_H

#include <tui/Mode.h>

class DevelopMode : public Mode
{
public:
  DevelopMode(Communication::Communicator& communicator, unsigned int zCoordinate);
  ~DevelopMode() override = default;
  virtual void start() override;

    enum WorldShape {SQUARE, RECTANGLE, CIRCLE=3};
    enum WorldColor {ALL, YELLOW, RED, GREEN, BLUE, BLACK, WHITE, WOOD};

private:
    WorldShape shapeKeywordToWorldInterface(Keyword keyword);
    WorldColor colorKeywordToWorldInterface(Keyword keyword);
    void handleFindRectangle(const std::string& color);
    void handleFindSquare(const std::string& color);
    void handleFindCircle(const std::string& color);
    void goToPosition(const std::string& positionStr);
    void createShapeChoiceMenu(const std::vector<tui::Shape>& shapes);

    unsigned int standardZ;
};

#endif //TUI_INCLUDE_TUI_DEVELOPMODE_H
