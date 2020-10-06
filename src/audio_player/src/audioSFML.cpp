//
// Created by lorenzo on 29/09/20.
//

#include "audioSFML.h"
#include <SFML/Audio.hpp>
int main()
{
    sf::SoundBuffer buffer;
    if (!buffer.loadFromFile("/home/lorenzo/RoboTheatreWS/src/audio_player/audio_files/blade_runner.wav"))
        return -1;
    return 0;
}