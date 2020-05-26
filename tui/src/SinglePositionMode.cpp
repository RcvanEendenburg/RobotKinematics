//
// Created by derk on 25-5-20.
//

#include <tui/SinglePositionMode.h>

SinglePositionMode::SinglePositionMode(Communication::Communicator &communicator) : PickUpBlockMode(communicator)
{

}

void SinglePositionMode::handleShape(std::unique_ptr<tui::Shape> shape)
{
    communicator.goToPosition(shape->points.x, shape->points.y, shape->points.z, shape->rotation, 0);
}