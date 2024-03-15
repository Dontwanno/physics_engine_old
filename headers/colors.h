//
// Created by twang on 30/06/2023.
//

#ifndef PHYSICS_ENGINE_COLORS_H
#define PHYSICS_ENGINE_COLORS_H

#include <vector>
#include <string>
#include <glm/glm.hpp>

class Colors {
public:
    std::vector<std::string> hex_colors = {"f94144","f3722c","f8961e","f9844a","f9c74f","90be6d","43aa8b","4d908e","577590","277da1","264653","2a9d8f","e9c46a","f4a261","e76f51"};
    std::vector<glm::vec3> glm_colors = {glm::vec3(249,65,68),glm::vec3(243,114,44),glm::vec3(248,150,30),glm::vec3(249,132,74),glm::vec3(249,199,79),
                                         glm::vec3(144,190,109),glm::vec3(67,170,139),glm::vec3(77,144,142),glm::vec3(87,117,144),glm::vec3(39,125,161),
                                         glm::vec3(38,70,83),glm::vec3(42,157,143),glm::vec3(233,196,106),glm::vec3(244,162,97),glm::vec3(231,111,81),
                                         glm::vec3(217,73,41), glm::vec3(242,149,68), glm::vec3(95, 148,136)};

};

#endif //PHYSICS_ENGINE_COLORS_H
