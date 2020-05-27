//
// Created by derk on 25-5-20.
//

#ifndef SINGLEPOSITIONMODE_H
#define SINGLEPOSITIONMODE_H

#include <tui/PickUpBlockMode.h>


class SinglePositionMode : public PickUpBlockMode
{
public:
    SinglePositionMode(Communication::Communicator& communicator);
    ~SinglePositionMode() override = default;
private:
    void handleShape(std::unique_ptr<tui::Shape> shape) override;
};


#endif //SINGLEPOSITIONMODE_H
