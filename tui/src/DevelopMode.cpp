//
// Created by derk on 27-4-20.
//

#include <tui/DevelopMode.h>

DevelopMode::DevelopMode(Communication::Communicator& communicator, unsigned int zCoordinate) : Mode(communicator), standardZ(zCoordinate)
{
    addOperationWithArgument(keywordToString(Keyword::Goto), [this](const std::string& arg){return goToPosition(arg);});
    addOperationWithArgument(keywordToString(Keyword::Rectangle), [this](const std::string& arg){handleFindRectangle(arg);});
    addOperationWithArgument(keywordToString(Keyword::Square), [this](const std::string& arg){handleFindSquare(arg);});
    addOperationWithArgument(keywordToString(Keyword::Circle), [this](const std::string& arg){handleFindCircle(arg);});
}

void DevelopMode::goToPosition(const std::string& positionStr)
{
    std::string str = positionStr;
    auto trimLeftBracketPos = str.find_first_not_of('(');
    str = str.substr(trimLeftBracketPos != std::string::npos ? trimLeftBracketPos : 0);
    auto trimRightBracketPos = str.find_last_not_of(')');
    str = str.substr(0, trimRightBracketPos != std::string::npos ? trimRightBracketPos + 1 : str.size() - 1);
    std::string token;
    std::vector<unsigned int> coordinates;
    while(true)
    {
        std::size_t pos = str.find(',');
        token = str.substr(0, pos);
        str.erase(0, pos + 1);
        coordinates.push_back(static_cast<unsigned int>(std::stoi(token)));
        if(pos == std::string::npos) break;
    }

    if(coordinates.size() != 3)
        throw std::runtime_error("This is not a valid coordinate!");

    communicator.goToPosition(coordinates.at(0)/100.0, coordinates.at(1)/100.0, coordinates.at(2)/100.0);
}

void DevelopMode::handleFindRectangle(const std::string& color)
{
    try
    {
        const Keyword colorKeyword = stringToKeyword(color);
        const WorldShape worldShape = shapeKeywordToWorldInterface(Keyword::Rectangle);
        const WorldColor worldColor = colorKeywordToWorldInterface(colorKeyword);
        auto shapes = communicator.findShapes(worldShape, worldColor);
        if(shapes.size() > 1) createShapeChoiceMenu(shapes);
        if(shapes.size() == 1) communicator.goToPosition(shapes[0].points.x/100.0, shapes[0].points.y/100.0, standardZ/100.0);
    }
    catch(std::exception& e)
    {
        throw;
    }
}

void DevelopMode::handleFindSquare(const std::string& color)
{
    try
    {
        const Keyword colorKeyword = stringToKeyword(color);
        const WorldShape worldShape = shapeKeywordToWorldInterface(Keyword::Square);
        const WorldColor worldColor = colorKeywordToWorldInterface(colorKeyword);
        auto shapes = communicator.findShapes(worldShape, worldColor);
        if(shapes.size() > 1) createShapeChoiceMenu(shapes);
        if(shapes.size() == 1) communicator.goToPosition(shapes[0].points.x/100.0, shapes[0].points.y/100.0, standardZ/100.0);
    }
    catch(std::exception& e)
    {
        throw;
    }
}

DevelopMode::WorldShape DevelopMode::shapeKeywordToWorldInterface(Keyword keyword)
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

DevelopMode::WorldColor DevelopMode::colorKeywordToWorldInterface(Keyword keyword)
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

void DevelopMode::createShapeChoiceMenu(const std::vector<tui::Shape>& shapes)
{
    char choice;
    logger.log(Utilities::LogLevel::Raw, "Found multiple shapes! Make a choice from the following shapes.");
    while(true)
    {
        for (unsigned int i = 0; i < shapes.size(); ++i) {
            logger.log(Utilities::LogLevel::Raw, "%d -> center: (%f, %f), rotation: %d", i, shapes[i].points.x,
                       shapes[i].points.y, shapes[i].rotation);
        }
        logger.log(Utilities::LogLevel::Raw, "q: exit menu");
        std::cin >> choice;
        if(choice == 'q') break;
        else if(isdigit(choice))
        {
            //TODO: make an option for rotation
            int index = static_cast<int>(choice) - 48;
            if(index < shapes.size())
            {
                communicator.goToPosition(shapes[index].points.x/100.0, shapes[index].points.y/100.0, standardZ/100.0);
                break;
            }
        }

    }

}

void DevelopMode::handleFindCircle(const std::string& color)
{
    try
    {
        const Keyword colorKeyword = stringToKeyword(color);
        const WorldShape worldShape = shapeKeywordToWorldInterface(Keyword::Circle);
        const WorldColor worldColor = colorKeywordToWorldInterface(colorKeyword);
        auto shapes = communicator.findShapes(worldShape, worldColor);
        if(shapes.size() > 1) createShapeChoiceMenu(shapes);
        if(shapes.size() == 1) communicator.goToPosition(shapes[0].points.x/100.0, shapes[0].points.y/100.0, standardZ/100.0);
    }
    catch(std::exception& e)
    {
        throw;
    }
}

void DevelopMode::start()
{
    setWelcomeMessage("Use goto (x,y,z) to go to a specific position.");
    setExitMessage("Exiting developer mode...");
    Mode::start();
}