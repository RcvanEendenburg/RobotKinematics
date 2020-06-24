//
// Created by derk on 25-5-20.
//

#include <tui/SequenceMode.h>

SequenceMode::SequenceMode(Communication::Communicator &communicator, double aGripperMaxOpenDistance, double aGripperOpeningHeight) :
FindShapeMode(communicator), gripperMaxOpenDistance(aGripperMaxOpenDistance), gripperOpeningHeight(aGripperOpeningHeight)
{

}

void SequenceMode::handleShape(std::unique_ptr<tui::Shape> shape)
{
    std::chrono::seconds sleep_time(8);
    logger.log(Utilities::LogLevel::Debug, "Going to a point above the block...");
    if(!communicator.goToPosition(shape->points.x, shape->points.y + gripperOpeningHeight, shape->points.z, shape->rotation, 0))
        return;
    std::this_thread::sleep_for(sleep_time);

    logger.log(Utilities::LogLevel::Debug, "Opening gripper...");
    if(!communicator.goToPosition(shape->points.x, shape->points.y + gripperOpeningHeight, shape->points.z, shape->rotation,
                              gripperMaxOpenDistance))
        return;
    std::this_thread::sleep_for(sleep_time);

    logger.log(Utilities::LogLevel::Debug, "Going down to the block position...");
    if(!communicator.goToPosition(shape->points.x, shape->points.y, shape->points.z, shape->rotation, gripperMaxOpenDistance))
        return;
    std::this_thread::sleep_for(sleep_time);

    logger.log(Utilities::LogLevel::Debug, "Closing gripper...");
    if(!communicator.goToPosition(shape->points.x, shape->points.y, shape->points.z, shape->rotation, 10))
        return;
    std::this_thread::sleep_for(sleep_time);

    logger.log(Utilities::LogLevel::Debug, "Lifting block...");
    if(!communicator.goToPosition(shape->points.x, shape->points.y + gripperOpeningHeight, shape->points.z, shape->rotation, 10))
        return;
    std::this_thread::sleep_for(sleep_time);

    auto shapes = communicator.findShapes(WorldShape::CIRCLE, WorldColor::WHITE);
    std::unique_ptr<tui::Shape> chosenShape = nullptr;
    if(shapes.size() > 1) chosenShape = std::move(retrieveShape(shapes));
    if(shapes.size() == 1) chosenShape = std::make_unique<tui::Shape>(shapes[0]);
    if(chosenShape)
    {
        logger.log(Utilities::LogLevel::Debug, "Going to the chosen goal...");
        communicator.goToPosition(chosenShape->points.x, chosenShape->points.y, chosenShape->points.z,
        chosenShape->rotation, shape->width);
        std::this_thread::sleep_for(sleep_time);

        logger.log(Utilities::LogLevel::Debug, "Opening gripper...");
        communicator.goToPosition(chosenShape->points.x, chosenShape->points.y, chosenShape->points.z,
                                  chosenShape->rotation, gripperMaxOpenDistance);
        std::this_thread::sleep_for(sleep_time);

        logger.log(Utilities::LogLevel::Debug, "Going to a point above the block...");
        communicator.goToPosition(chosenShape->points.x, chosenShape->points.y + gripperOpeningHeight, chosenShape->points.z,
                                  chosenShape->rotation, gripperMaxOpenDistance);
        std::this_thread::sleep_for(sleep_time);
    }
    else
    {
        logger.log(Utilities::LogLevel::Error, "No goal defined!");
    }
}
