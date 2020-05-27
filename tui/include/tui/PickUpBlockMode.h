//
// Created by derk on 15-5-20.
//

#ifndef PICKUPBLOCKMODE_H
#define PICKUPBLOCKMODE_H

#include <tui/Mode.h>

class PickUpBlockMode : public Mode
{
public:
    PickUpBlockMode(Communication::Communicator& communicator);
    ~PickUpBlockMode() override = default;
    virtual void start() override;

    enum WorldShape {SQUARE, RECTANGLE, CIRCLE=3};
    enum WorldColor {ALL, YELLOW, RED, GREEN, BLUE, BLACK, WHITE, WOOD};

protected:
    virtual void handleShape(std::unique_ptr<tui::Shape>) = 0;
    std::unique_ptr<tui::Shape> retrieveShape(const std::vector<tui::Shape>& shapes);

private:
    WorldShape shapeKeywordToWorldInterface(Keyword keyword);
    WorldColor colorKeywordToWorldInterface(Keyword keyword);
    void handleFindRectangle(const std::string& color);
    void handleFindSquare(const std::string& color);
    void handleFindCircle(const std::string& color);
};


#endif //PICKUPBLOCKMODE_H
