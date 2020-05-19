//
// Created by derk on 18-5-20.
//

#include <tui/ChooseShapeMode.h>

ChooseShapeMode::ChooseShapeMode(Communication::Communicator &communicator, const std::vector<tui::Shape> &theShapes) :
Mode(communicator), shapes(theShapes)
{
    for (unsigned int i = 0; i < shapes.size(); ++i)
    {
        addSingleOperation(std::to_string(i), [this, &communicator, i]()
        {
            communicator.goToPosition(shapes[i].points.x, shapes[i].points.y, shapes[i].points.z,
                                                                                             shapes[i].rotation);
            started = false;
        });
    }
}

void ChooseShapeMode::start()
{
    std::stringstream ss;
    for (unsigned int i = 0; i < shapes.size(); ++i) {
        ss << i << " -> center: (" << shapes[i].points.x << ", " << shapes[i].points.y << ", " <<
        shapes[i].points.z << "), rotation: " << shapes[i].rotation << std::endl;
    }
    ss << "exit -> go back to the interactive mode" << std::endl;
    setWelcomeMessage(ss.str());
    setExitMessage("Exiting choose shape menu...");
    Mode::start();
}