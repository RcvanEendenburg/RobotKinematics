#include <utilities/IniParser.h>
#include <iostream>

namespace Utilities
{
IniParser::IniParser(std::string aFilename)
    : file(), filename(std::move(aFilename)), lastSection("") {}

void
IniParser::parse()
{
    file.open(filename);
    if (!file.is_open())
        throw std::runtime_error("Could not open file: " + filename);

    std::string line;
    while (std::getline(file, line))
        parseLine(line);
    file.close();
}

void
IniParser::parseLine(const std::string &line)
{
    std::string trimmedLine = trim(line);

    if (trimmedLine.empty())
        return;
    if (isComment(trimmedLine))
        return;

    if (!isSection(trimmedLine))
        parameters.insert(std::make_pair(lastSection, getVariable(trimmedLine)));
    else
        lastSection = getSectionName(trimmedLine);
}

const std::string
IniParser::trim(const std::string &line)
{
    std::string str = line;
    auto quoteOpen = str.end();
    auto quoteClose = str.end();

    for (auto it = str.begin(); it != str.end(); ++it)
    {
        if (*it == '"')
        {
            if (quoteOpen == str.end())
                quoteOpen = it;
            else
            {
                quoteClose = it;
                break;
            }
        }
    }
    if (quoteOpen != str.end() && quoteClose == str.end())
        throw std::runtime_error(
            "Opening quotes do not match the closing ones! Incorrect config file!");
    str.erase(std::remove(str.begin(), quoteOpen, ' '), quoteOpen);
    str.erase(std::remove(quoteClose, str.end(), ' '), str.end());
    str.erase(std::remove(str.begin(), str.end(), '"'), str.end());
    str.erase(std::remove(str.begin(), str.end(), '\r'), str.end());
    return str;
}

bool
IniParser::isComment(const std::string &line)
{
    const char firstCharacter = *line.begin();
    return firstCharacter == '#' || firstCharacter == ';';
}

const std::string
IniParser::getVariableName(const std::string &line)
{
    std::string name;
    for (char c : line)
    {
        if (c != '=' && c != ':')
            name.push_back(c);
        else
            break;
    }
    return name;
}

const std::string
IniParser::getSectionName(const std::string &line)
{
    std::string name;
    bool bracketFound = false;
    for (char c : line)
    {
        if (c == '[' && !bracketFound)
            bracketFound = true;
        else if (c != ']')
            name.push_back(c);
        else if (bracketFound)
            break;
    }
    return name;
}

bool
IniParser::isSection(const std::string &line)
{
    return *line.begin() == '[';
}

std::pair<std::string, std::string>
IniParser::getVariable(const std::string &line)
{
    return std::make_pair(getVariableName(line), getVariableValue(line));
}

const std::string
IniParser::getVariableValue(const std::string &line)
{
    std::string value;
    bool delimiterHit = false;
    for (char c : line)
    {
        if ((c == '=' || c == ':') && !delimiterHit)
            delimiterHit = true;
        else if (delimiterHit)
            value.push_back(c);
    }
    return value;
}

const std::string
IniParser::searchParameter(const std::string &sectionName,
                           const std::string &parameter) const
{
    auto range = parameters.equal_range(sectionName);
    for (auto i = range.first; i != range.second; ++i)
    {
        if (i->second.first == parameter)
            return i->second.second;
    }
    throw std::runtime_error("Parameter " + parameter + " in " + sectionName +
        " not found in file");
}
} // namespace Utilities
