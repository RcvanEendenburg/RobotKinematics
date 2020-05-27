//
// Created by derk on 15-5-20.
//

#ifndef FINDSHAPEMODE_H
#define FINDSHAPEMODE_H

#include <tui/Mode.h>

/**
 * This mode is not usable on its own, but defines a mode which can handle "shape color" operations.
 * There is a designated function to handle the found shape(s) based on the operation.
 */
class FindShapeMode : public Mode
{
public:
    explicit FindShapeMode(Communication::Communicator& communicator);
    ~FindShapeMode() override = default;

    /**
     * @see Mode::start
     */
    void start() override;

    ///@{
    /** These enumerations correspond to the World interface.
     * Don't change unless you have changed/going to change world as well! **/
    enum WorldShape {SQUARE, RECTANGLE, CIRCLE=3};
    enum WorldColor {ALL, YELLOW, RED, GREEN, BLUE, BLACK, WHITE, WOOD};
    ///@}

protected:
    /**
     * When the shape is found, this method will be called.
     */
    virtual void handleShape(std::unique_ptr<tui::Shape>) = 0;

    /**
     * Retrieves the shape from the found shapes. It will start the menu through starting ChooseShapeMode.
     * @param shapes The found shapes.
     * @return The chosen shape if the user have chosen a shape.
     * @return nullptr if the user exited the ChooseShapeMode.
     */
    std::unique_ptr<tui::Shape> retrieveShape(const std::vector<tui::Shape>& shapes);

private:
    ///@{
    /** Conversion from keyword to world interface **/
    WorldShape shapeKeywordToWorldInterface(Keyword keyword);
    WorldColor colorKeywordToWorldInterface(Keyword keyword);
    ///@}

    ///@{
    /** Handle functions for the different shapes. **/
    void handleFindRectangle(const std::string& color);
    void handleFindSquare(const std::string& color);
    void handleFindCircle(const std::string& color);
    ///@}
};


#endif //FINDSHAPEMODE_H
