//
// Created by derk on 25-5-20.
//

#ifndef SEQUENCEMODE_H
#define SEQUENCEMODE_H

#include <tui/FindShapeMode.h>

/**
 * The sequence mode is about picking up the shape and putting it in the goal.
 *
 * The general set of actions should be:
 *
 * 1. Retrieve shape (for instance based on command "rectangle red").
 * 2. Move to the (x,y,z) position of the shape, but a little bit higher so the arm has some space to open the gripper.
 * 3. Open gripper, but keep same position.
 * 4. Move to the (x,y,z) position of the shape.
 * 5. Close the gripper, but keep same position.
 * 6. Find goal based on command "circle white".
 * 7. Open gripper, but keep same position.
 * 8. Move a little bit higher, so the arm has some space to open the gripper.
 * 9. Open the gripper and keep same position.
 *
 * Notice that the first step is handled in the FindShapeMode and not specifically in this mode.
 * Notice that from step 7, it is assumed that the goal exists. If multiple goals exists, the user will be presented
 * with a menu of shapes to choose from.
 */
class SequenceMode : public FindShapeMode
{
public:
    SequenceMode(Communication::Communicator& communicator, double aGripperMaxOpenDistance, double aGripperOpeningHeight);
    ~SequenceMode() override = default;
private:
    /**
      * @see FindShapeMode::handleShape
      */
    void handleShape(std::unique_ptr<tui::Shape> shape) override;

    double gripperMaxOpenDistance;
    double gripperOpeningHeight;
};


#endif //SEQUENCEMODE_H
