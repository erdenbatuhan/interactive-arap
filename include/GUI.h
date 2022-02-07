/**
 * Reference repositories:
 *  - https://github.com/libigl/libigl/blob/main/tutorial/708_Picking/main.cpp
 */

#ifndef _GUI_H_
#define _GUI_H_

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>

class GUI {
public:
    explicit GUI(const std::string&);

    // Launches the GLFW viewer
    void launchViewer();
private:
    // Vertices, colors and faces of the model
    Eigen::MatrixXd m_vertices, m_colors;
    Eigen::MatrixXi m_faces;

    // GLFW viewer
    igl::opengl::glfw::Viewer m_viewer;

    // Anchor vertex ids to be passed to ARAP algorithm
    std::vector<int> m_anchorVertexIds;

    // Returns the mouse position
    Eigen::Vector2f getMousePosition();

    // Converts the camera position of the vertex to a world position
    Eigen::Vector3f convertCameraToWorldPosition(int vertexId);

    // Handles mouse click event
    void handleMouseClick();
};

#endif //_GUI_H_
