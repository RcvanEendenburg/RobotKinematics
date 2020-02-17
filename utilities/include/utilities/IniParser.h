#ifndef INI_PARSER_H
#define INI_PARSER_H

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

namespace Utilities
{
class IniParser
{
public:
    /**
     * @brief Private constructor with the name of the file that will be loaded.
     * @param aFilename
     */
    explicit IniParser(std::string aFilename);

    /**
     * @brief Function to parse the .ini file. All parameters will be saved in a
     * multimap.
     */
    void
    parse();

    /**
     * @brief Function to get a parameter through the name of a section and the
     * name of a parameter.
     * @param sectionName
     * @param parameter
     * @return the parameter value
     */
    template<typename T>
    T
    get(const std::string &sectionName,
        const std::string &parameter) const;

    ~IniParser() = default;

private:
    /**
     * @brief This function will parse a line.
     * @param line
     */
    void
    parseLine(const std::string &line);

    /**
     * @brief This function will remove all whitespace of the given string.
     * @param line
     * @return the trimmed string
     */
    static const std::string
    trim(const std::string &line);

    /**
     * @brief This function checks whether or not the given line is a comment.
     * @param line
     * @return true if the given line is a comment, false if the given line isn't
     * a comment
     */
    static bool
    isComment(const std::string &line);

    /**
     * @brief Function to get a variable
     * @param line
     * @return pair with the name of the variable as key and the value of the
     * variable as value
     */
    static std::pair<std::string, std::string>
    getVariable(const std::string &line);

    /**
     * @brief Extracts the name of a variable from the given line.
     * @param line
     * @return the name of a variable
     */
    static const std::string
    getVariableName(const std::string &line);

    /**
     * @brief Extracts the value of a variable from the given line.
     * @param line
     * @return the value of a variable
     */
    static const std::string
    getVariableValue(const std::string &line);

    /**
     * @brief Extracts the name of a section from the given line.
     * @param line
     * @return the name of a section
     */
    static const std::string
    getSectionName(const std::string &line);

    /**
     * @brief Checks whether or not the line is a section.
     * @param line
     * @return true if the line is a section, false if the line isn't a section
     */
    static bool
    isSection(const std::string &line);

    /**
     * @brief Function to search for a parameter in the multimap.
     * @param sectionName
     * @param parameter
     * @return the parameter
     */
    const std::string
    searchParameter(const std::string &sectionName,
                    const std::string &parameter) const;

    std::ifstream file;
    std::string filename;
    std::multimap<std::string, std::pair<std::string, std::string>> parameters;
    std::string lastSection;
};

/**
 * @brief Function to get the value of a parameter as an integer type.
 * @param sectionName
 * @param parameter
 * @return the value of the parameter
 */
template<typename T>
inline T
IniParser::get(const std::string &sectionName,
               const std::string &parameter) const
{
    throw std::runtime_error("Not implemented!");
}

/**
 * @brief Function to get the value of a parameter as a float type.
 * @param sectionName
 * @param parameter
 * @return the value of the parameter
 */
template<>
inline float
IniParser::get(const std::string &sectionName,
               const std::string &parameter) const
{
    return std::stof(searchParameter(sectionName, parameter));
}

/**
 * @brief Function to get the value of a parameter as a double type.
 * @param sectionName
 * @param parameter
 * @return the value of the parameter
 */
template<>
inline double
IniParser::get(const std::string &sectionName,
               const std::string &parameter) const
{
    return std::stod(searchParameter(sectionName, parameter));
}

/**
 * @brief Function to get the value of a parameter as a string.
 * @param sectionName
 * @param parameter
 * @return the value of the parameter
 */
template<>
inline std::string
IniParser::get(const std::string &sectionName,
               const std::string &parameter) const
{
    return searchParameter(sectionName, parameter);
}

/**
 * @brief Function to get the value of a parameter as a string.
 * @param sectionName
 * @param parameter
 * @return the value of the parameter
 */
template<>
inline bool
IniParser::get(const std::string &sectionName,
               const std::string &parameter) const
{
    return searchParameter(sectionName, parameter) == "true";
}

template<>
inline int
IniParser::get(const std::string &sectionName,
               const std::string &parameter) const
{
    return std::stoi(searchParameter(sectionName, parameter));
}

/**
 * @brief Function to get the value of a parameter as an long integer.
 * @param sectionName
 * @param parameter
 * @return the value of the parameter
 */
template<>
inline long
IniParser::get(const std::string &sectionName,
               const std::string &parameter) const
{
    return std::stol(searchParameter(sectionName, parameter));
}

/**
 * @brief Function to get the value of a parameter as an long long integer.
 * @param sectionName
 * @param parameter
 * @return the value of the parameter
 */
template<>
inline unsigned long long
IniParser::get(const std::string &sectionName,
               const std::string &parameter) const
{
    return std::stoull(searchParameter(sectionName, parameter));
}
} // namespace Utilities

#endif // INI_PARSER_H
