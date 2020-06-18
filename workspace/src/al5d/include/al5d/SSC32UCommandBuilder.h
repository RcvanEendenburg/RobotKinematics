#ifndef SSC32UCOMMANDBUILDER_H
#define SSC32UCOMMANDBUILDER_H

template<std::size_t S>
class SSC32UCommandBuilder
{
public:
    SSC32UCommandBuilder() = default;
    ~SSC32UCommandBuilder() = default;

    static std::string
    createCommand(const std::array<Joint, S> &joints, unsigned short time);

};

#include "SSC32UCommandBuilder.ipp"

#endif //SC32UCOMMANDBUILDER_H
