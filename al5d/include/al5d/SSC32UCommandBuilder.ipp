
template<std::size_t S>
std::string
SSC32UCommandBuilder<S>::createCommand(const std::array<Joint, S> &joints, unsigned short time)
{
    std::string command;
    for (std::size_t i = 0; i < S; ++i)
    {
        if(joints[i].getPwm() == 0)
            continue;
        command += '#' + std::to_string(joints[i].getChannel());
        command += 'P' + std::to_string(joints[i].getPwm());
        auto speed = joints[i].getSpeed();
        if (speed != 0)
            command += 'S' + speed;
    }
    command += 'T' + std::to_string(time);
    command += '\r';
    return command;
}
