//
// Created by derk on 25-5-20.
//

#include <tui/SequenceMode.h>

SequenceMode::SequenceMode(Communication::Communicator &communicator, double aGripperOpeningHeight) :
PickUpBlockMode(communicator), gripperOpeningHeight(aGripperOpeningHeight)
{

}

void SequenceMode::handleShape(std::unique_ptr<tui::Shape> shape)
{
    logger.log(Utilities::LogLevel::Debug, "Going to a point above the block...");
    logger.log(Utilities::LogLevel::Warning, "Block width not defined!");
    communicator.goToPosition(shape->points.x, shape->points.y - gripperOpeningHeight, shape->points.z, shape->rotation, 0);

    logger.log(Utilities::LogLevel::Debug, "Opening gripper...");
    logger.log(Utilities::LogLevel::Warning, "Gripper should open now but is not defined!");
    communicator.goToPosition(shape->points.x, shape->points.y - gripperOpeningHeight, shape->points.z, shape->rotation, 0);

    logger.log(Utilities::LogLevel::Debug, "Going down to the block position...");
    logger.log(Utilities::LogLevel::Warning, "Should keep gripper open but it's not defined!");
    communicator.goToPosition(shape->points.x, shape->points.y, shape->points.z, shape->rotation, 0);

    logger.log(Utilities::LogLevel::Debug, "Closing gripper...");
    logger.log(Utilities::LogLevel::Warning, "Gripper should close now but it's not defined!");
    communicator.goToPosition(shape->points.x, shape->points.y, shape->points.z, shape->rotation, 0);

    auto shapes = communicator.findShapes(WorldShape::CIRCLE, WorldColor::WHITE);
    std::unique_ptr<tui::Shape> chosenShape = nullptr;
    if(shapes.size() > 1) chosenShape = std::move(retrieveShape(shapes));
    if(shapes.size() == 1) chosenShape = std::make_unique<tui::Shape>(shapes[0]);
    if(chosenShape)
    {
        logger.log(Utilities::LogLevel::Debug, "Going to the chosen goal...");
        logger.log(Utilities::LogLevel::Warning, "Should keep gripper close but it's not defined!");
        communicator.goToPosition(chosenShape->points.x, chosenShape->points.y, chosenShape->points.z,
        chosenShape->rotation, 0);

        logger.log(Utilities::LogLevel::Debug, "Opening gripper...");
        logger.log(Utilities::LogLevel::Warning, "Should keep gripper open but it's not defined!");
        communicator.goToPosition(chosenShape->points.x, chosenShape->points.y, chosenShape->points.z,
                                  chosenShape->rotation, 0);

        logger.log(Utilities::LogLevel::Debug, "Going to a point above the block...");
        communicator.goToPosition(chosenShape->points.x, chosenShape->points.y - gripperOpeningHeight, chosenShape->points.z,
                                  chosenShape->rotation, 0);
    }
    else
    {
        logger.log(Utilities::LogLevel::Error, "No goal defined!");
    }
}
