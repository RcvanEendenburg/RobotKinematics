//
// Created by derk on 18-5-20.
//

#include <tui/ChooseShapeMode.h>

ChooseShapeMode::ChooseShapeMode(Communication::Communicator &communicator, const std::vector<tui::Shape> &theShapes) :
Mode(communicator), shapes(theShapes), chosenShape(nullptr)
{
    for (unsigned int i = 0; i < shapes.size(); ++i)
    {
        addSingleOperation(std::to_string(i), [this, i]()
        {
            chosenShape = std::make_unique<tui::Shape>(shapes[i]);
            started = false;
        });
    }
}

std::unique_ptr<tui::Shape> ChooseShapeMode::retrieveShape()
{
    std::thread([this](){
    std::unique_lock<std::mutex> lock(retrieveShapeMutex);
    retrieveShapeCv.wait(lock, [this]{return chosenShape || !started;});
    }).detach();
    return std::move(chosenShape);
}


void ChooseShapeMode::start()
{
    std::stringstream ss;
    for (unsigned int i = 0; i < shapes.size(); ++i) {
        ss << i << " -> center: (" << shapes[i].points.x << ", " << shapes[i].points.y << ", " <<
        shapes[i].points.z << "), rotation: " << shapes[i].rotation << std::endl;
    }
    ss << "exit -> go back to the previous mode" << std::endl;
    setWelcomeMessage(ss.str());
    setExitMessage("Exiting choose shape menu...");
    Mode::start();
}