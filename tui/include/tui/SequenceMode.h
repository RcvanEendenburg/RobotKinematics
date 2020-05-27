//
// Created by derk on 25-5-20.
//

#ifndef SEQUENCEMODE_H
#define SEQUENCEMODE_H

#include <tui/PickUpBlockMode.h>

class SequenceMode : public PickUpBlockMode
{
public:
    SequenceMode(Communication::Communicator& communicator, double aGripperOpeningHeight);
    ~SequenceMode() override = default;
private:
    void handleShape(std::unique_ptr<tui::Shape> shape) override;
    double gripperOpeningHeight;
};


#endif //SEQUENCEMODE_H
