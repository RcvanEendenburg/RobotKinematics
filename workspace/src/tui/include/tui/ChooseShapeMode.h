//
// Created by derk on 18-5-20.
//

#ifndef CHOOSESHAPEMODE_H
#define CHOOSESHAPEMODE_H

#include <tui/Mode.h>

/**
 * The choose shape mode writes a menu to the display based on the defined shapes.
 */
class ChooseShapeMode : public Mode
{
public:
    /**
     * Sets up the mode.
     * @param communicator A reference to the communicator.
     * @param theShapes The shapes to be put in menu.
     */
    ChooseShapeMode(Communication::Communicator& communicator, const std::vector<tui::Shape>& theShapes);
    ~ChooseShapeMode() override = default;

    /**
     * Starts another thread and waits until the shape has been selected or the mode has been ended.
     * @return The shape if the user selected a shape.
     * @return nullptr if the user exited the mode.
     */
    std::unique_ptr<tui::Shape> retrieveShape();

    /**
     * @see Mode::start
     */
    void start() override;
private:
    const std::vector<tui::Shape> &shapes;
    std::unique_ptr<tui::Shape> chosenShape;
    std::condition_variable retrieveShapeCv;
    std::mutex retrieveShapeMutex;
};


#endif //CHOOSESHAPEMODE_H
