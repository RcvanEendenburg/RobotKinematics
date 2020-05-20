//
// Created by derk on 15-5-20.
//

#ifndef INTERACTIVEMODE_H
#define INTERACTIVEMODE_H

#include <tui/Mode.h>

class InteractiveMode : public Mode
{
public:
    InteractiveMode(Communication::Communicator& communicator);
    ~InteractiveMode() override = default;
    virtual void start() override;

    enum WorldShape {SQUARE, RECTANGLE, CIRCLE=3};
    enum WorldColor {ALL, YELLOW, RED, GREEN, BLUE, BLACK, WHITE, WOOD};

private:
    WorldShape shapeKeywordToWorldInterface(Keyword keyword);
    WorldColor colorKeywordToWorldInterface(Keyword keyword);
    void handleFindRectangle(const std::string& color);
    void handleFindSquare(const std::string& color);
    void handleFindCircle(const std::string& color);
    void createShapeChoiceMenu(const std::vector<tui::Shape>& shapes);
    void goToParkPosition();
};


#endif //INTERACTIVEMODE_H
