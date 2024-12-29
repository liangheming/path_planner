#include "teb_path_follower/commons.h"
#include <iostream>
int main(int argc, char **argv)
{
    float angel = 3.64;
    float ret = normalize_theta(angel);
    std::cout << "angel:" << angel << " ret:" << ret << std::endl;
}