//
// Created by derk on 18-5-20.
//

#ifndef CHOOSESHAPEMODE_H
#define CHOOSESHAPEMODE_H

#include <tui/Mode.h>

class ChooseShapeMode : public Mode
{
public:
    ChooseShapeMode(Communication::Communicator& communicator, const std::vector<tui::Shape>& theShapes);
    ~ChooseShapeMode() override = default;

    void start() override;
private:
    const std::vector<tui::Shape> &shapes;
};


#endif //CHOOSESHAPEMODE_H
