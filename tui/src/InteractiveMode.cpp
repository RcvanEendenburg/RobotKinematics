//
// Created by derk on 27-4-20.
//

#include <tui/InteractiveMode.h>
#include <tui/ChooseShapeMode.h>

InteractiveMode::InteractiveMode(Communication::Communicator& communicator) : Mode(communicator)
{
    addOperationWithArgument(keywordToString(Keyword::Rectangle), [this](const std::string& arg){handleFindRectangle(arg);});
    addOperationWithArgument(keywordToString(Keyword::Square), [this](const std::string& arg){handleFindSquare(arg);});
    addOperationWithArgument(keywordToString(Keyword::Circle), [this](const std::string& arg){handleFindCircle(arg);});
    addSingleOperation("Park", [this](){goToParkPosition();});
}

void InteractiveMode::goToParkPosition()
{
   // communicator.GoToPark()
}


void InteractiveMode::handleFindRectangle(const std::string& color)
{
    try
    {
        const Keyword colorKeyword = stringToKeyword(color);
        const WorldShape worldShape = shapeKeywordToWorldInterface(Keyword::Rectangle);
        const WorldColor worldColor = colorKeywordToWorldInterface(colorKeyword);
        auto shapes = communicator.findShapes(worldShape, worldColor);
        if(shapes.size() > 1) createShapeChoiceMenu(shapes);
        if(shapes.size() == 1) communicator.goToPosition(shapes[0].points.x, shapes[0].points.y,
                                                         shapes[0].points.z, shapes[0].rotation);
    }
    catch(std::exception& e)
    {
        throw;
    }
}

void InteractiveMode::handleFindSquare(const std::string& color)
{
    try
    {
        const Keyword colorKeyword = stringToKeyword(color);
        const WorldShape worldShape = shapeKeywordToWorldInterface(Keyword::Square);
        const WorldColor worldColor = colorKeywordToWorldInterface(colorKeyword);
        auto shapes = communicator.findShapes(worldShape, worldColor);
        if(shapes.size() > 1) createShapeChoiceMenu(shapes);
        if(shapes.size() == 1) communicator.goToPosition(shapes[0].points.x, shapes[0].points.y,
                                                         shapes[0].points.z, shapes[0].rotation);
    }
    catch(std::exception& e)
    {
        throw;
    }
}

InteractiveMode::WorldShape InteractiveMode::shapeKeywordToWorldInterface(Keyword keyword)
{
    switch(keyword)
    {
    case Keyword::Rectangle:
        return WorldShape::RECTANGLE;
    case Keyword::Square:
        return WorldShape::SQUARE;
    case Keyword::Circle:
        return WorldShape::CIRCLE;
    default:
        throw std::runtime_error("This keyword is not a shape");
    }
}

InteractiveMode::WorldColor InteractiveMode::colorKeywordToWorldInterface(Keyword keyword)
{
    switch(keyword)
    {
    case Keyword::All:
        return WorldColor::ALL;
    case Keyword::Yellow:
        return WorldColor::YELLOW;
    case Keyword::Red:
        return WorldColor::RED;
    case Keyword::Green:
        return WorldColor::GREEN;
    case Keyword::Blue:
        return WorldColor::BLUE;
    case Keyword::Black:
        return WorldColor::BLACK;
    case Keyword::White:
        return WorldColor::WHITE;
    case Keyword::Wood:
        return WorldColor::WOOD;
    default:
        throw std::runtime_error("This keyword is not a color");
    }
}

void InteractiveMode::createShapeChoiceMenu(const std::vector<tui::Shape>& shapes)
{
    ChooseShapeMode chooseShapeMode(communicator, shapes);
    chooseShapeMode.start();
}

void InteractiveMode::handleFindCircle(const std::string& color)
{
    try
    {
        const Keyword colorKeyword = stringToKeyword(color);
        const WorldShape worldShape = shapeKeywordToWorldInterface(Keyword::Circle);
        const WorldColor worldColor = colorKeywordToWorldInterface(colorKeyword);
        auto shapes = communicator.findShapes(worldShape, worldColor);
        if(shapes.size() > 1) createShapeChoiceMenu(shapes);
        if(shapes.size() == 1) communicator.goToPosition(shapes[0].points.x, shapes[0].points.y,
                                                         shapes[0].points.z, shapes[0].rotation);
    }
    catch(std::exception& e)
    {
        throw;
    }
}

void InteractiveMode::start()
{
    std::stringstream ss;
    ss << "Use $shape $color to pick up a block." << std::endl;
    ss << "$shape can consist of the following shapes: " << keywordToString(Keyword::Rectangle) << ", " <<
    keywordToString(Keyword::Square) << " or " << keywordToString(Keyword::Circle) << "." << std::endl;
    ss << "$color can consist of the following colors: " << keywordToString(Keyword::Wood) << ", " <<
    keywordToString(Keyword::Black) << ", " << keywordToString(Keyword::Blue) << ", " <<
    keywordToString(Keyword::Green) << ", " << keywordToString(Keyword::Yellow) << ", " <<
    keywordToString(Keyword::Red) << " or " << keywordToString(Keyword::White) << std::endl;
    ss << "In addition to this, it is also possible to search for all colors with $shape " <<
    keywordToString(Keyword::All) << std::endl;
    ss << "In order to set the robot back in its park position, the command Park, can be used" << std::endl;
    setWelcomeMessage(ss.str());
    setExitMessage("Exiting interactive mode...");
    Mode::start();
}