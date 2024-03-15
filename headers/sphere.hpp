//
// Created by twang on 18/06/2023.
//

#ifndef PHYSICS_ENGINE_SPHERE_HPP
#define PHYSICS_ENGINE_SPHERE_HPP

#define _USE_MATH_DEFINES
#include <vector>
#include <cmath>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <cmath>

#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923

// your framework of choice here

class Sphere {
// declare vertices and indices
    std::vector<float> sphereVertices;
    std::vector<unsigned int> sphereIndices;

public:
    Sphere(float radius, int resolution) {
        setupSphere(radius, resolution);
        setupMesh();
    }

void draw() {
    // draw sphere
    glBindVertexArray(sphereVAO);
    glDrawElements(GL_TRIANGLES, sphereIndices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}


private:


void setupSphere(float SPHERE_RADIUS, int SPHERE_RESOLUTION) {
    for (int i = 0; i <= SPHERE_RESOLUTION; i++) {
        float phi = (float) i / (float) SPHERE_RESOLUTION;
        phi = phi * M_PI;

        for (int j = 0; j <= SPHERE_RESOLUTION; j++) {
            float theta = (float) j / (float) SPHERE_RESOLUTION;
            theta = theta * 2.0f * M_PI;

            // Calculate the x, y, and z coordinates of the point on the sphere
            float x = SPHERE_RADIUS * sinf(phi) * cosf(theta);
            float y = SPHERE_RADIUS * sinf(phi) * sinf(theta);
            float z = SPHERE_RADIUS * cosf(phi);

            // Calculate the normal vector for the point on the sphere
            float nx = x / SPHERE_RADIUS;
            float ny = y / SPHERE_RADIUS;
            float nz = z / SPHERE_RADIUS;

            // Add the coordinates and normal vector to the sphereVertices vector
            sphereVertices.push_back(x);
            sphereVertices.push_back(y);
            sphereVertices.push_back(z);
            sphereVertices.push_back(nx);
            sphereVertices.push_back(ny);
            sphereVertices.push_back(nz);
        }
    }

    // Generate the indices for the sphere
    for (int i = 0; i < SPHERE_RESOLUTION; i++) {
        for (int j = 0; j < SPHERE_RESOLUTION; j++) {
            unsigned int first = (i * (SPHERE_RESOLUTION + 1)) + j;
            unsigned int second = first + SPHERE_RESOLUTION + 1;

            sphereIndices.push_back(first);
            sphereIndices.push_back(second);
            sphereIndices.push_back(first + 1);

            sphereIndices.push_back(second);
            sphereIndices.push_back(second + 1);
            sphereIndices.push_back(first + 1);
        }
    }
}

unsigned int sphereVAO, sphereVBO, sphereEBO;

void setupMesh() {
    // create buffers/arrays
    glGenVertexArrays(1, &sphereVAO);
    glGenBuffers(1, &sphereVBO);
    glGenBuffers(1, &sphereEBO);

    glBindVertexArray(sphereVAO);
    // load data into vertex buffers
    glBindBuffer(GL_ARRAY_BUFFER, sphereVBO);
    glBufferData(GL_ARRAY_BUFFER, sphereVertices.size() * sizeof(float), &sphereVertices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphereEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphereIndices.size() * sizeof(unsigned int), &sphereIndices[0],
                 GL_STATIC_DRAW);

    // set the vertex attribute pointers
    // vertex Positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) 0);
    // vertex normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) (3 * sizeof(float)));

    glBindVertexArray(0);
}


};

#endif //PHYSICS_ENGINE_SPHERE_HPP
