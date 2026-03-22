


#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "shader.h"
#include "camera.h"

#include <iostream>
#include <vector>

// window settings
const unsigned int SCR_WIDTH = 1200;
const unsigned int SCR_HEIGHT = 800;

// cameras for 4 viewports
Camera mainCamera(glm::vec3(0.0f, 5.0f, 15.0f)); 
Camera birdEyeCamera(glm::vec3(0.0f, 20.0f, 0.1f)); 
Camera followCamera(glm::vec3(0.0f, 5.0f, 15.0f));  
Camera frontCamera(glm::vec3(0.0f, 2.0f, 10.0f));   

float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// Toggles state
bool dirLightOn = true;
bool pointLightOn = true;
bool spotLightOn = true;
bool ambientOn = true;
bool diffuseOn = true;
bool specularOn = true;
bool masterLightOn = true;
bool mainLightOn = true;
bool usePhong = true;
bool fanOn = false;
bool singleViewport = false;

// Debounce state
bool key1_pressed = false, key2_pressed = false, key3_pressed = false;
bool key5_pressed = false, key6_pressed = false, key7_pressed = false;
bool keyL_pressed = false, keyM_pressed = false, keyG_pressed = false;
bool key8_pressed = false, key9_pressed = false, keyV_pressed = false;

// Animations
float fanAngle = 0.0f;
float robotBaseAngle = 0.0f;
float robotElbowAngle = 0.0f;
float bindRotAngle = 0.0f;
float armReachAngle = -30.0f;
float gripperSpread = 0.3f; // gripper finger spread (0.1=closed, 0.7=open)

struct ConveyorLine {
    glm::vec3 start;
    glm::vec3 end;
    bool isHorizontal;
};
std::vector<ConveyorLine> lines = {
    // Horizontal lines (along X axis) — LOWER level y=0.8
    {{-40, 0.8f, -20}, {40, 0.8f, -20}, true},
    {{-40, 0.8f, -10}, {40, 0.8f, -10}, true},
    {{-40, 0.8f,   0}, {40, 0.8f,   0}, true},
    {{-40, 0.8f,  10}, {40, 0.8f,  10}, true},
    {{-40, 0.8f,  20}, {40, 0.8f,  20}, true},
    // Vertical lines (along Z axis) — UPPER level y=3.0 to pass clearly OVER horizontal boxes
    {{-20, 3.0f, -40}, {-20, 3.0f, 40}, false},
    {{-10, 3.0f, -40}, {-10, 3.0f, 40}, false},
    {{  0, 3.0f, -40}, {  0, 3.0f, 40}, false},
    {{ 10, 3.0f, -40}, { 10, 3.0f, 40}, false},
    {{ 20, 3.0f, -40}, { 20, 3.0f, 40}, false}
};

enum BoxStage { RAW = 0, PAINTED = 1, BOUND = 2 };

struct GridBox {
    int lineIndex;
    float distance;
    BoxStage stage;
};
std::vector<GridBox> gridBoxes;

// Basic custom perspective
glm::mat4 customPerspective(float fovRadians, float aspect, float zNear, float zFar) {
    glm::mat4 result(0.0f);
    float tanHalfFovy = tan(fovRadians / 2.0f);
    
    result[0][0] = 1.0f / (aspect * tanHalfFovy);
    result[1][1] = 1.0f / (tanHalfFovy);
    result[2][2] = -(zFar + zNear) / (zFar - zNear);
    result[2][3] = -1.0f;
    result[3][2] = -(2.0f * zFar * zNear) / (zFar - zNear);
    
    return result;
}

// Binding arm: full body FIXED, only gripper fingers open/close
void drawBindingArm(Shader &shader, glm::vec3 basePos, float baseRotY, unsigned int darkTex, unsigned int lightTex, float gripperSpread) {
    // Fixed base pedestal
    glBindTexture(GL_TEXTURE_2D, darkTex);
    glm::mat4 base = glm::translate(glm::mat4(1.0f), basePos);
    base = glm::rotate(base, glm::radians(baseRotY), glm::vec3(0,1,0));
    shader.setMat4("model", glm::scale(base, glm::vec3(2.0f, 0.6f, 2.0f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    glBindTexture(GL_TEXTURE_2D, lightTex);
    glm::mat4 column = glm::translate(base, glm::vec3(0.0f, 0.6f, 0.0f));
    shader.setMat4("model", glm::scale(column, glm::vec3(0.8f, 2.4f, 0.8f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    glBindTexture(GL_TEXTURE_2D, darkTex);
    glm::mat4 elbow = glm::translate(column, glm::vec3(0.0f, 2.4f, 0.0f));
    shader.setMat4("model", glm::scale(elbow, glm::vec3(1.0f, 0.5f, 1.0f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Arm extends horizontally toward belt (in -Z direction)
    glm::mat4 reach = glm::translate(elbow, glm::vec3(0.0f, 0.5f, -1.0f));
    shader.setMat4("model", glm::scale(reach, glm::vec3(0.5f, 0.5f, 2.0f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Wrist plate at end of horizontal reach
    glBindTexture(GL_TEXTURE_2D, lightTex);
    glm::mat4 wrist = glm::translate(elbow, glm::vec3(0.0f, 0.5f, -2.1f));
    shader.setMat4("model", glm::scale(wrist, glm::vec3(0.8f, 0.3f, 0.3f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // GRIPPER FINGERS — only these move (spread opens/closes)
    glBindTexture(GL_TEXTURE_2D, darkTex);
    glm::mat4 f1 = glm::translate(wrist, glm::vec3(-gripperSpread, -0.4f, -0.15f));
    shader.setMat4("model", glm::scale(f1, glm::vec3(0.15f, 0.8f, 0.2f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glm::mat4 f2 = glm::translate(wrist, glm::vec3( gripperSpread, -0.4f, -0.15f));
    shader.setMat4("model", glm::scale(f2, glm::vec3(0.15f, 0.8f, 0.2f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

// Shelf arm: reaches upward to place boxes. shoulderAngle animates up/down.
void drawShelfArm(Shader &shader, glm::vec3 basePos, float baseRotY, float shoulderAngle, unsigned int darkTex, unsigned int lightTex) {
    glBindTexture(GL_TEXTURE_2D, darkTex);
    glm::mat4 base = glm::translate(glm::mat4(1.0f), basePos);
    base = glm::rotate(base, glm::radians(baseRotY), glm::vec3(0,1,0));
    shader.setMat4("model", glm::scale(base, glm::vec3(2.0f, 0.6f, 2.0f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    glBindTexture(GL_TEXTURE_2D, lightTex);
    glm::mat4 turret = glm::translate(base, glm::vec3(0.0f, 0.6f, 0.0f));
    shader.setMat4("model", glm::scale(turret, glm::vec3(1.2f, 0.8f, 1.2f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    glBindTexture(GL_TEXTURE_2D, darkTex);
    glm::mat4 shoulderOrigin = glm::translate(turret, glm::vec3(0.0f, 0.8f, 0.0f));
    shoulderOrigin = glm::rotate(shoulderOrigin, glm::radians(shoulderAngle), glm::vec3(1,0,0));
    glm::mat4 upperArm = glm::translate(shoulderOrigin, glm::vec3(0.0f, 1.5f, 0.0f));
    shader.setMat4("model", glm::scale(upperArm, glm::vec3(0.7f, 3.0f, 0.7f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    glBindTexture(GL_TEXTURE_2D, lightTex);
    glm::mat4 elbow = glm::translate(shoulderOrigin, glm::vec3(0.0f, 3.0f, 0.0f));
    shader.setMat4("model", glm::scale(elbow, glm::vec3(1.0f, 0.5f, 1.0f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Forearm angles down
    glBindTexture(GL_TEXTURE_2D, darkTex);
    glm::mat4 foreArm = glm::translate(elbow, glm::vec3(0.0f, 0.5f, 0.0f));
    foreArm = glm::rotate(foreArm, glm::radians(-shoulderAngle*0.5f), glm::vec3(1,0,0));
    glm::mat4 foreArmDraw = glm::translate(foreArm, glm::vec3(0.0f, 1.0f, 0.0f));
    shader.setMat4("model", glm::scale(foreArmDraw, glm::vec3(0.5f, 2.0f, 0.5f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Simple gripper at end
    glBindTexture(GL_TEXTURE_2D, lightTex);
    glm::mat4 grip = glm::translate(foreArm, glm::vec3(0.0f, 2.1f, 0.0f));
    shader.setMat4("model", glm::scale(grip, glm::vec3(0.9f, 0.2f, 0.9f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void processInput(GLFWwindow *window);
unsigned int loadTexture(const char *path, unsigned char r=150, unsigned char g=150, unsigned char b=150);
void renderScene(Shader &shader, unsigned int VAO, unsigned int boxTex, unsigned int conveyorTex, unsigned int floorTex, unsigned int wallTex, unsigned int blueTex, glm::mat4 view, glm::mat4 projection, glm::vec3 camPos);

int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "FactoryFlow 3D", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glEnable(GL_DEPTH_TEST);

    mainCamera.Mode = REALISTIC;
    birdEyeCamera.Mode = BIRD_EYE;
    followCamera.Mode = FOLLOW;
    frontCamera.Mode = ASSIGNMENT;
    frontCamera.Position = glm::vec3(0.0f, 7.0f, 15.0f);
    frontCamera.Pitch = -15.0f; // Look slightly down

    Shader phongShader("shaders/vertexShaderForPhongShading.vs", "shaders/fragmentShaderForPhongShading.fs");
    Shader gouraudShader("shaders/vertexShaderForGouraudShading.vs", "shaders/fragmentShaderForGouraudShading.fs");

    float vertices[] = {
        // positions          // normals           // texture coords
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,

        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,

        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  1.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
        -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  0.0f,
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,

         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  1.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
         0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,

        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f,
         0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  1.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f,

        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f
    };

    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    unsigned int boxTexture = loadTexture("textures/box.jpg", 160, 100, 50); // Wooden brown
    unsigned int conveyorTexture = loadTexture("textures/conveyor.jpg", 30, 30, 30); // Very Dark gray
    unsigned int floorTexture = loadTexture("textures/floor.jpg", 180, 180, 180); // Light gray
    unsigned int wallTexture = loadTexture("textures/wall.jpg", 120, 140, 160); // Slate blue/gray
    unsigned int blueTexture = loadTexture("textures/blue.jpg", 0, 150, 255); // Cyan-blue

    // Populate hundreds of boxes densely on all grid lines
    for(int i=0; i<lines.size(); i++) {
        float len = glm::length(lines[i].end - lines[i].start);
        int numBoxes = (int)(len / 4.0f); // Space them exactly per line length
        float spacing = len / (float)numBoxes;
        for(int b=0; b<numBoxes; b++) {
            gridBoxes.push_back({i, b * spacing, RAW});
        }
    }

    while (!glfwWindowShouldClose(window))
    {
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        // Update animations
        if (fanOn) fanAngle += 150.0f * deltaTime;
        robotBaseAngle += 20.0f * deltaTime;
        robotElbowAngle = sin(glfwGetTime() * 2.0f) * 30.0f;
        // Gripper cycles: open(0.7) -> close(0.1) -> hold -> repeat
        gripperSpread = 0.4f + 0.35f * sin(glfwGetTime() * 2.5f); // oscillates 0.05..0.75

        // --- BOX PROCESSING STAGE MACHINE ---
        // Paint chamber on line 7 (upper vertical, X=0, Z from -40 to 40)
        // distance 40 = center of 80-unit line = Z=0
        float paintZoneCenter = 40.0f;
        float paintRadius = 5.0f;
        // Robot binding arms on line 2 (lower horizontal, Z=0, X from -40 to 40)
        // 3 arms at X=-10, X=5, X=20 => distances 30, 45, 60 on that line
        float bindPositions[3] = {30.0f, 45.0f, 60.0f};
        float bindRadius = 4.0f;
        for(auto &b : gridBoxes) {
            if(b.lineIndex == 7) { // upper vertical belt at X=0
                if(b.distance >= paintZoneCenter - paintRadius &&
                   b.distance <= paintZoneCenter + paintRadius) {
                    b.stage = PAINTED;
                }
                if(b.distance < 2.0f) b.stage = RAW;
            }
            if(b.lineIndex == 2) { // lower horizontal belt at Z=0
                for(int arm=0; arm<3; arm++) {
                    if(b.distance >= bindPositions[arm] - bindRadius &&
                       b.distance <= bindPositions[arm] + bindRadius) {
                        b.stage = BOUND;
                    }
                }
                if(b.distance < 2.0f) b.stage = RAW;
            }
        }
        // Animate arm reach (oscillate toward the belt and back)
        armReachAngle = -30.0f + sin(glfwGetTime() * 3.0f) * 28.0f;
        
        // Box animation along rectilinear grid
        float speed = 1.5f; // Slower, realistic conveyor speed
        for (int i = 0; i < gridBoxes.size(); i++) {
            gridBoxes[i].distance += speed * deltaTime;
            float lineLen = glm::length(lines[gridBoxes[i].lineIndex].end - lines[gridBoxes[i].lineIndex].start);
            if (gridBoxes[i].distance >= lineLen) {
                gridBoxes[i].distance -= lineLen; // Perfect seamless wrapping math
            }
        }

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Follow a random active box
        if(!gridBoxes.empty()) {
            glm::vec3 dir = glm::normalize(lines[0].end - lines[0].start);
            glm::vec3 tgtPos = lines[0].start + dir * gridBoxes[0].distance;
            followCamera.SetTarget(tgtPos);
        }

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        float aspect = singleViewport ? (float)width / (float)height : (float)(width / 2) / (float)(height / 2);
        glm::mat4 projection = customPerspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);

        if (singleViewport) {
            glViewport(0, 0, width, height);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, mainCamera.GetViewMatrix(), projection, mainCamera.Position);
        } else {
            // 1. Top-Left Viewport (Main Interactive Camera)
            glViewport(0, height / 2, width / 2, height / 2);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, mainCamera.GetViewMatrix(), projection, mainCamera.Position);

            // 2. Top-Right Viewport (Bird's Eye)
            glViewport(width / 2, height / 2, width / 2, height / 2);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, birdEyeCamera.GetViewMatrix(), projection, birdEyeCamera.Position);

            // 3. Bottom-Left Viewport (Follow Camera)
            glViewport(0, 0, width / 2, height / 2);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, followCamera.GetViewMatrix(), projection, followCamera.Position);

            // 4. Bottom-Right Viewport (Static Front Camera)
            glViewport(width / 2, 0, width / 2, height / 2);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, frontCamera.GetViewMatrix(), projection, frontCamera.Position);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glfwTerminate();
    return 0;
}

void renderScene(Shader &shader, unsigned int VAO, unsigned int boxTex, unsigned int conveyorTex, unsigned int floorTex, unsigned int wallTex, unsigned int blueTex, glm::mat4 view, glm::mat4 projection, glm::vec3 camPos) {
    shader.use();
    shader.setMat4("projection", projection);
    shader.setMat4("view", view);
    
    // Explicitly set material uniforms
    shader.setInt("material.diffuse", 0);
    shader.setInt("material.specular", 0); // use diffuse map for specular as well for simplicity
    shader.setFloat("material.shininess", 32.0f);
    
    shader.setBool("ambientOn", masterLightOn && ambientOn);
    shader.setBool("diffuseOn", masterLightOn && diffuseOn);
    shader.setBool("specularOn", masterLightOn && specularOn);
    shader.setBool("dirLightOn", masterLightOn && dirLightOn);
    shader.setBool("pointLightOn", masterLightOn && pointLightOn);
    shader.setBool("spotLightOn", masterLightOn && spotLightOn);
    
    // Setup Directional Light
    shader.setVec3("dirLight.direction", -0.2f, -1.0f, -0.3f);
    shader.setVec3("dirLight.ambient", 0.3f, 0.3f, 0.3f);
    shader.setVec3("dirLight.diffuse", 0.5f, 0.5f, 0.5f);
    shader.setVec3("dirLight.specular", 0.5f, 0.5f, 0.5f);
    
    // Setup Point Lights (1 green, 1 red for factory lighting)
    shader.setVec3("pointLights[0].position", -10.0f, 8.0f, 0.0f);
    shader.setVec3("pointLights[0].ambient", 0.0f, 0.1f, 0.3f);
    shader.setVec3("pointLights[0].diffuse", 0.3f, 0.6f, 1.0f); // Industrial bluish
    shader.setVec3("pointLights[0].specular", 1.0f, 1.0f, 1.0f);
    shader.setFloat("pointLights[0].constant", 1.0f);
    shader.setFloat("pointLights[0].linear", 0.045f);
    shader.setFloat("pointLights[0].quadratic", 0.0075f);

    shader.setVec3("pointLights[1].position", 10.0f, 8.0f, 0.0f);
    shader.setVec3("pointLights[1].ambient", 0.0f, 0.1f, 0.3f);
    shader.setVec3("pointLights[1].diffuse", 0.3f, 0.6f, 1.0f);
    shader.setVec3("pointLights[1].specular", 1.0f, 1.0f, 1.0f);
    shader.setFloat("pointLights[1].constant", 1.0f);
    shader.setFloat("pointLights[1].linear", 0.045f);
    shader.setFloat("pointLights[1].quadratic", 0.0075f);

    // Setup Main Roof Light (pointLights[2])
    shader.setVec3("pointLights[2].position", 0.0f, 9.8f, 0.0f);
    if(mainLightOn && masterLightOn && pointLightOn) {
        shader.setVec3("pointLights[2].ambient", 0.2f, 0.2f, 0.2f);
        shader.setVec3("pointLights[2].diffuse", 1.0f, 1.0f, 1.0f);
        shader.setVec3("pointLights[2].specular", 1.0f, 1.0f, 1.0f);
    } else {
        shader.setVec3("pointLights[2].ambient", 0.0f, 0.0f, 0.0f);
        shader.setVec3("pointLights[2].diffuse", 0.0f, 0.0f, 0.0f);
        shader.setVec3("pointLights[2].specular", 0.0f, 0.0f, 0.0f);
    }
    shader.setFloat("pointLights[2].constant", 1.0f);
    shader.setFloat("pointLights[2].linear", 0.045f);
    shader.setFloat("pointLights[2].quadratic", 0.0075f);
    
    // Disable pointLight 3 to avoid artifacting
    shader.setVec3("pointLights[3].ambient", 0.0f, 0.0f, 0.0f);
    shader.setVec3("pointLights[3].diffuse", 0.0f, 0.0f, 0.0f);
    shader.setVec3("pointLights[3].specular", 0.0f, 0.0f, 0.0f);
    shader.setFloat("pointLights[3].constant", 1.0f);
    shader.setFloat("pointLights[3].linear", 0.09f);
    shader.setFloat("pointLights[3].quadratic", 0.032f);

    // Setup Spot Light (Camera flashlight)
    shader.setVec3("spotLight.position", mainCamera.Position);
    shader.setVec3("spotLight.direction", mainCamera.Front);
    shader.setVec3("spotLight.ambient", 0.0f, 0.0f, 0.0f);
    shader.setVec3("spotLight.diffuse", 1.0f, 1.0f, 1.0f);
    shader.setVec3("spotLight.specular", 1.0f, 1.0f, 1.0f);
    shader.setFloat("spotLight.constant", 1.0f);
    shader.setFloat("spotLight.linear", 0.09f);
    shader.setFloat("spotLight.quadratic", 0.032f);
    shader.setFloat("spotLight.cutOff", glm::cos(glm::radians(12.5f)));
    shader.setFloat("spotLight.outerCutOff", glm::cos(glm::radians(15.0f)));

    shader.setVec3("viewPos", camPos);

    glBindVertexArray(VAO);
    glActiveTexture(GL_TEXTURE0);

    // 1. DRAW SPRAWLING CONVEYOR GRID NETWORK
    for(size_t i=0; i<lines.size(); i++) {
        glm::vec3 dir = lines[i].end - lines[i].start;
        float len = glm::length(dir);
        glm::vec3 center = lines[i].start + dir * 0.5f;
        float angle = atan2(dir.x, dir.z);
        
        glm::mat4 model = glm::translate(glm::mat4(1.0f), center);
        model = glm::rotate(model, angle, glm::vec3(0,1,0));
        
        // Main flat belt track
        glBindTexture(GL_TEXTURE_2D, conveyorTex);
        glm::mat4 trackModel = glm::scale(model, glm::vec3(2.8f, 0.2f, len)); // belt
        shader.setMat4("model", trackModel);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // Glowing Blue Guide Rails
        glBindTexture(GL_TEXTURE_2D, blueTex);
        glm::mat4 rail1 = glm::translate(model, glm::vec3(-1.45f, 0.15f, 0.0f));
        shader.setMat4("model", glm::scale(rail1, glm::vec3(0.1f, 0.3f, len)));
        glDrawArrays(GL_TRIANGLES, 0, 36);
        
        glm::mat4 rail2 = glm::translate(model, glm::vec3(1.45f, 0.15f, 0.0f));
        shader.setMat4("model", glm::scale(rail2, glm::vec3(0.1f, 0.3f, len)));
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // Concrete support legs repeating along track length
        glBindTexture(GL_TEXTURE_2D, wallTex);
        int numLegs = (int)(len / 4.0f);
        for(int j=0; j<=numLegs; j++) {
            float dist = j * 4.0f - len/2.0f;
            glm::mat4 legCenter = glm::translate(model, glm::vec3(0.0f, 0.0f, dist));
            
            glm::mat4 legL = glm::translate(legCenter, glm::vec3(-1.3f, -0.85f, 0.0f));
            shader.setMat4("model", glm::scale(legL, glm::vec3(0.2f, 1.7f, 0.2f)));
            glDrawArrays(GL_TRIANGLES, 0, 36);
            
            glm::mat4 legR = glm::translate(legCenter, glm::vec3(1.3f, -0.85f, 0.0f));
            shader.setMat4("model", glm::scale(legR, glm::vec3(0.2f, 1.7f, 0.2f)));
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
    }

    // 2. DRAW MASSIVE BOX POPULATION (Animated)
    for(size_t i=0; i<gridBoxes.size(); i++) {
        glm::vec3 dir = glm::normalize(lines[gridBoxes[i].lineIndex].end - lines[gridBoxes[i].lineIndex].start);
        glm::vec3 pos = lines[gridBoxes[i].lineIndex].start + dir * gridBoxes[i].distance;
        pos.y = lines[gridBoxes[i].lineIndex].start.y + 0.6f;
        float angle = atan2(dir.x, dir.z);

        // Choose color/texture by stage
        if(gridBoxes[i].stage == PAINTED || gridBoxes[i].stage == BOUND) {
            glBindTexture(GL_TEXTURE_2D, blueTex);
        } else {
            glBindTexture(GL_TEXTURE_2D, boxTex);
        }

        glm::mat4 model = glm::translate(glm::mat4(1.0f), pos);
        model = glm::rotate(model, angle, glm::vec3(0,1,0));
        model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));
        shader.setMat4("model", model);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // If BOUND: draw 3 thin hemp-coloured rope bands over the box
        if(gridBoxes[i].stage == BOUND) {
            glBindTexture(GL_TEXTURE_2D, wallTex); // tan/grey band colour
            float offsets[3] = {-0.3f, 0.0f, 0.3f};
            for(int r=0; r<3; r++) {
                glm::mat4 rope = glm::translate(glm::mat4(1.0f), pos + glm::vec3(0.0f, offsets[r], 0.0f));
                rope = glm::rotate(rope, angle, glm::vec3(0,1,0));
                rope = glm::scale(rope, glm::vec3(1.05f, 0.12f, 1.05f));
                shader.setMat4("model", rope);
                glDrawArrays(GL_TRIANGLES, 0, 36);
            }
        }
    }

    // 3. DRAW MASSIVE SHELVING — Left Half = FRESH boxes | Right Half = BOUND (blue) boxes
    glBindTexture(GL_TEXTURE_2D, conveyorTex); // Dark racking iron
    for(int s=0; s<4; s++) {
        float shelfX = -45.0f + s * 30.0f;
        float shelfZ = -45.0f;
        bool isBoundSide = (s >= 2); // right 2 towers hold finished bound boxes

        // Horizontal shelf tiers
        for(int y=0; y<5; y++) {
            glBindTexture(GL_TEXTURE_2D, conveyorTex);
            glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(shelfX, y*3.0f, shelfZ));
            model = glm::scale(model, glm::vec3(26.0f, 0.2f, 4.0f));
            shader.setMat4("model", model);
            glDrawArrays(GL_TRIANGLES, 0, 36);

            // Boxes on shelf: fresh-brown on left, blue-bound on right
            for(float bx = -12.0f; bx <= 12.0f; bx += 1.5f) {
                if((int)(bx*shelfX+y) % 5 == 0) continue; // natural gaps

                if(isBoundSide) {
                    // Blue painted + bound box
                    glBindTexture(GL_TEXTURE_2D, blueTex);
                    glm::mat4 bModel = glm::translate(glm::mat4(1.0f), glm::vec3(shelfX+bx, y*3.0f+0.6f, shelfZ));
                    shader.setMat4("model", glm::scale(bModel, glm::vec3(1.0f, 1.0f, 1.2f)));
                    glDrawArrays(GL_TRIANGLES, 0, 36);
                    // Rope bands
                    glBindTexture(GL_TEXTURE_2D, wallTex);
                    for(float ro : {-0.25f, 0.0f, 0.25f}) {
                        glm::mat4 rModel = glm::translate(glm::mat4(1.0f), glm::vec3(shelfX+bx, y*3.0f+0.6f+ro, shelfZ));
                        shader.setMat4("model", glm::scale(rModel, glm::vec3(1.05f, 0.1f, 1.25f)));
                        glDrawArrays(GL_TRIANGLES, 0, 36);
                    }
                } else {
                    // Fresh cardboard brown boxes
                    glBindTexture(GL_TEXTURE_2D, boxTex);
                    glm::mat4 bModel = glm::translate(glm::mat4(1.0f), glm::vec3(shelfX+bx, y*3.0f+0.6f, shelfZ));
                    shader.setMat4("model", glm::scale(bModel, glm::vec3(1.0f, 1.0f, 1.2f)));
                    glDrawArrays(GL_TRIANGLES, 0, 36);
                }
            }
        }

        // Vertical posts
        glBindTexture(GL_TEXTURE_2D, conveyorTex);
        for(float vx = -13.0f; vx <= 13.0f; vx += 13.0f) {
            glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(shelfX+vx, 6.0f, shelfZ));
            model = glm::scale(model, glm::vec3(0.5f, 12.0f, 4.0f));
            shader.setMat4("model", model);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
    }

    // 3.5 PAINT CHAMBER — on UPPER BELT (line 7 = vertical, X=0, positioned at Z=0)
    {
        // Main chamber shell (dark metallic industrial block) — stands OVER the upper belt
        glBindTexture(GL_TEXTURE_2D, conveyorTex);
        glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 4.6f, 0.0f)); // centred at Z=0
        model = glm::scale(model, glm::vec3(5.0f, 4.0f, 6.0f)); // spans the track width
        shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);

        // Entry/Exit arch openings (bright blue glowing slits)
        glBindTexture(GL_TEXTURE_2D, blueTex);
        model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 3.2f, 0.0f));
        model = glm::scale(model, glm::vec3(3.0f, 0.4f, 5.8f));
        shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);

        // Two top exhaust chimney pipes
        glBindTexture(GL_TEXTURE_2D, wallTex);
        for(float px : {-1.0f, 1.0f}) {
            model = glm::translate(glm::mat4(1.0f), glm::vec3(px, 7.8f, 0.0f));
            model = glm::scale(model, glm::vec3(0.5f, 2.5f, 0.5f));
            shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
        }
    }

    // 4a. BINDING ARM — 1 arm beside lower belt at Z=0 (near binding zone at X=-10)
    //     Body FIXED, only gripper opens/closes around the passing box
    drawBindingArm(shader, glm::vec3(-10.0f, 0.0f, 3.0f), 180.0f, conveyorTex, wallTex, gripperSpread);

    // 4b. SHELF PLACEMENT ARMS — 2 arms at the back shelves
    //     Left shelf arm (serves fresh cardboard boxes on left shelf towers)
    float shelfArmAngle = -30.0f + sin(glfwGetTime() * 1.5f) * 35.0f;
    drawShelfArm(shader, glm::vec3(-45.0f, 0.0f, -38.0f),  90.0f, shelfArmAngle, conveyorTex, wallTex);
    //     Right shelf arm (places bound blue boxes on right shelf towers)
    drawShelfArm(shader, glm::vec3( 50.0f, 0.0f, -38.0f), -90.0f, shelfArmAngle * 0.8f, conveyorTex, wallTex);

    // 5. DRAW MASSIVE WAREHOUSE ROOM (Floor, Concrete Pillars, Ceiling)
    glBindTexture(GL_TEXTURE_2D, floorTex);
    
    // Concrete Floor (100x100)
    glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -0.7f, 0.0f));
    model = glm::scale(model, glm::vec3(100.0f, 0.1f, 100.0f));
    shader.setMat4("model", model);
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Ceiling Base
    model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 15.0f, 0.0f));
    model = glm::scale(model, glm::vec3(100.0f, 0.1f, 100.0f));
    shader.setMat4("model", model);
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Large Main Concrete Pillars
    float pilX[6] = {-25.0f, 25.0f, -25.0f, 25.0f, 0.0f, 0.0f};
    float pilZ[6] = {-25.0f, -25.0f, 25.0f, 25.0f, -25.0f, 25.0f};
    for (int i=0; i<6; i++) {
        model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(pilX[i], 7.0f, pilZ[i]));
        model = glm::scale(model, glm::vec3(2.5f, 16.0f, 2.5f));
        shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    
    // Warehouse Walls Boundaries
    glBindTexture(GL_TEXTURE_2D, wallTex);
    // Back Wall
    model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 7.0f, -50.0f));
    model = glm::scale(model, glm::vec3(100.0f, 16.0f, 0.1f));
    shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    // Front Wall
    model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 7.0f, 50.0f));
    model = glm::scale(model, glm::vec3(100.0f, 16.0f, 0.1f));
    shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    // Left Wall
    model = glm::translate(glm::mat4(1.0f), glm::vec3(-50.0f, 7.0f, 0.0f));
    model = glm::scale(model, glm::vec3(0.1f, 16.0f, 100.0f));
    shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    // Right Wall
    model = glm::translate(glm::mat4(1.0f), glm::vec3(50.0f, 7.0f, 0.0f));
    model = glm::scale(model, glm::vec3(0.1f, 16.0f, 100.0f));
    shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
}

void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (mainCamera.Mode == REALISTIC || mainCamera.Mode == ASSIGNMENT) {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) mainCamera.ProcessKeyboard(FORWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) mainCamera.ProcessKeyboard(BACKWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) mainCamera.ProcessKeyboard(LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) mainCamera.ProcessKeyboard(RIGHT, deltaTime);
    }
    
    // Assignment mode specific controls
    if (mainCamera.Mode == ASSIGNMENT) {
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) mainCamera.ProcessKeyboard(UP, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) mainCamera.ProcessKeyboard(DOWN, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) mainCamera.ProcessKeyboard(PITCH_UP, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_Y) == GLFW_PRESS) mainCamera.ProcessKeyboard(YAW_LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) mainCamera.ProcessKeyboard(ROLL_LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) mainCamera.ProcessKeyboard(ROTATE_AROUND, deltaTime);
    }

    // Toggles logic
    #define DO_TOGGLE(KEY, STATE_VAR, PRESSED_VAR) \
        if (glfwGetKey(window, KEY) == GLFW_PRESS) { \
            if (!PRESSED_VAR) { STATE_VAR = !STATE_VAR; PRESSED_VAR = true; } \
        } else { PRESSED_VAR = false; }

    DO_TOGGLE(GLFW_KEY_1, dirLightOn, key1_pressed)
    DO_TOGGLE(GLFW_KEY_2, pointLightOn, key2_pressed)
    DO_TOGGLE(GLFW_KEY_3, spotLightOn, key3_pressed)
    DO_TOGGLE(GLFW_KEY_5, ambientOn, key5_pressed)
    DO_TOGGLE(GLFW_KEY_6, diffuseOn, key6_pressed)
    DO_TOGGLE(GLFW_KEY_7, specularOn, key7_pressed)
    DO_TOGGLE(GLFW_KEY_L, masterLightOn, keyL_pressed)
    DO_TOGGLE(GLFW_KEY_M, mainLightOn, keyM_pressed)
    DO_TOGGLE(GLFW_KEY_G, fanOn, keyG_pressed)
    DO_TOGGLE(GLFW_KEY_V, singleViewport, keyV_pressed)
    
    if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS) {
        if (!key8_pressed) { usePhong = true; key8_pressed = true; }
    } else { key8_pressed = false; }
    
    if (glfwGetKey(window, GLFW_KEY_9) == GLFW_PRESS) {
        if (!key9_pressed) { usePhong = false; key9_pressed = true; }
    } else { key9_pressed = false; }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // The screen is resized but viewports are calculated dynamically each frame
}

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos;

    lastX = xpos;
    lastY = ypos;

    mainCamera.ProcessMouseMovement(xoffset, yoffset);
}

unsigned int loadTexture(char const * path, unsigned char r, unsigned char g, unsigned char b)
{
    unsigned int textureID;
    glGenTextures(1, &textureID);
    
    int width, height, nrComponents;
    unsigned char *data = stbi_load(path, &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)      format = GL_RED;
        else if (nrComponents == 3) format = GL_RGB;
        else if (nrComponents == 4) format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
        
        // Solid color fallback texture to distinguish materials
        unsigned char fallback[] = {r, g, b, 255};
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, fallback);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }

    return textureID;
}
