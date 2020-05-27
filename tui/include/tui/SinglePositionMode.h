//
// Created by derk on 25-5-20.
//

#ifndef SINGLEPOSITIONMODE_H
#define SINGLEPOSITIONMODE_H

#include <tui/FindShapeMode.h>

/**
 * This mode is about moving the arm to a specific position based on the selected shape.
 */
class SinglePositionMode : public FindShapeMode
{
public:
    explicit SinglePositionMode(Communication::Communicator& communicator);
    ~SinglePositionMode() override = default;
private:
    /**
     * @see FindShapeMode::handleShape
     */
    void handleShape(std::unique_ptr<tui::Shape> shape) override;
};


#endif //SINGLEPOSITIONMODE_H
