//
// Created by derk on 25-5-20.
//

#include <tui/SequenceMode.h>

SequenceMode::SequenceMode(Communication::Communicator &communicator) : PickUpBlockMode(communicator)
{

}

void SequenceMode::handleShape(std::unique_ptr<tui::Shape> shape)
{
}
