//
// Created by twang on 18/06/2023.
//

#ifndef PHYSICS_ENGINE_CUBE_HPP
#define PHYSICS_ENGINE_CUBE_HPP

#include <glad/glad.h>

class Cube {
    glm::vec3 scale;

    public:
        Cube(glm::vec3 scale) {
            this->scale = scale;
            setupCube();
        }

        void draw() {
            glBindVertexArray(cubeVAO);
            glDrawArrays(GL_TRIANGLES, 0, 36);
            glBindVertexArray(0);
        }
    private:
        unsigned int VBO, cubeVAO;

        // set up vertex data (and buffer(s)) and configure vertex attributes
        // ------------------------------------------------------------------
        float vertices[216] = {
                -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
                0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
                0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
                0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
                -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
                -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,

                -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
                0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
                0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
                0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
                -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
                -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,

                -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
                -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
                -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
                -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
                -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
                -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,

                0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
                0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
                0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
                0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
                0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
                0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,

                -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
                0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
                0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
                0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
                -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
                -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,

                -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
                0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
                0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
                0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
                -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
                -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f
        };

    void setupCube() {
        // first, configure the cube's VAO (and VBO)
        glGenVertexArrays(1, &cubeVAO);
        glGenBuffers(1, &VBO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glBindVertexArray(cubeVAO);

        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        // normal attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
    }
};
#endif //PHYSICS_ENGINE_CUBE_HPP
