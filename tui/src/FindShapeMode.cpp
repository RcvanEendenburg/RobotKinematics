//
// Created by derk on 27-4-20.
//

#include <tui/FindShapeMode.h>
#include <tui/ChooseShapeMode.h>

FindShapeMode::FindShapeMode(Communication::Communicator& communicator) : Mode(communicator)
{
    addOperationWithArgument(keywordToString(Keyword::Rectangle), [this](const std::string& arg){handleFindRectangle(arg);});
    addOperationWithArgument(keywordToString(Keyword::Square), [this](const std::string& arg){handleFindSquare(arg);});
    addOperationWithArgument(keywordToString(Keyword::Circle), [this](const std::string& arg){handleFindCircle(arg);});
}

void FindShapeMode::handleFindRectangle(const std::string& color)
{
    try
    {
        std::unique_ptr<tui::Shape> chosenShape = nullptr;
        const Keyword colorKeyword = stringToKeyword(color);
        const WorldShape worldShape = shapeKeywordToWorldInterface(Keyword::Square);
        const WorldColor worldColor = colorKeywordToWorldInterface(colorKeyword);
        auto shapes = communicator.findShapes(worldShape, worldColor);
        if(shapes.size() > 1) chosenShape = std::move(retrieveShape(shapes));
        if(shapes.size() == 1) chosenShape = std::make_unique<tui::Shape>(shapes[0]);
        if(chosenShape) handleShape(std::move(chosenShape));
        else logger.log(Utilities::LogLevel::Warning, "No shapes found!");
    }
    catch(std::exception& e)
    {
        throw;
    }
}

void FindShapeMode::handleFindSquare(const std::string& color)
{
    try
    {
        std::unique_ptr<tui::Shape> chosenShape = nullptr;
        const Keyword colorKeyword = stringToKeyword(color);
        const WorldShape worldShape = shapeKeywordToWorldInterface(Keyword::Square);
        const WorldColor worldColor = colorKeywordToWorldInterface(colorKeyword);
        auto shapes = communicator.findShapes(worldShape, worldColor);
        if(shapes.size() > 1) chosenShape = std::move(retrieveShape(shapes));
        if(shapes.size() == 1) chosenShape = std::make_unique<tui::Shape>(shapes[0]);
        if(chosenShape) handleShape(std::move(chosenShape));
        else logger.log(Utilities::LogLevel::Warning, "No shapes found!");
    }
    catch(std::exception& e)
    {
        throw;
    }
}

FindShapeMode::WorldShape FindShapeMode::shapeKeywordToWorldInterface(Keyword keyword)
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

FindShapeMode::WorldColor FindShapeMode::colorKeywordToWorldInterface(Keyword keyword)
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

std::unique_ptr<tui::Shape> FindShapeMode::retrieveShape(const std::vector<tui::Shape>& shapes)
{
    ChooseShapeMode chooseShapeMode(communicator, shapes);
    chooseShapeMode.start();
    return std::move(chooseShapeMode.retrieveShape());
}

void FindShapeMode::handleFindCircle(const std::string& color)
{
    try
    {
        std::unique_ptr<tui::Shape> chosenShape = nullptr;
        const Keyword colorKeyword = stringToKeyword(color);
        const WorldShape worldShape = shapeKeywordToWorldInterface(Keyword::Circle);
        const WorldColor worldColor = colorKeywordToWorldInterface(colorKeyword);
        auto shapes = communicator.findShapes(worldShape, worldColor);
        if(shapes.size() > 1) chosenShape = std::move(retrieveShape(shapes));
        if(shapes.size() == 1) chosenShape = std::make_unique<tui::Shape>(shapes[0]);
        if(chosenShape) handleShape(std::move(chosenShape));
        else logger.log(Utilities::LogLevel::Warning, "No shapes found!");
    }
    catch(std::exception& e)
    {
        throw;
    }
}

void FindShapeMode::start()
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
    setWelcomeMessage(ss.str());
    setExitMessage("Exiting current mode...");
    Mode::start();
}