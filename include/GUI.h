/**
 * Project: Interactive ARAP
 * File:    GUI.h
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#ifndef _GUI_H_
#define _GUI_H_

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>

#include <map>
#include <vector>

#define INVALID_VERTEX_ID -1

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

    // Selections stored
    std::map<int, int> m_selectedAnchorVertexId; // Each selected face stores the closest vertex to the selection
    int m_currentAnchorVertexId = INVALID_VERTEX_ID; // The closest vertex to the latest selection is stored for ARAP

    // State variables that is keeping track of the state
    bool m_mouseDownBeingRecorded = false; // True when mouse down event is being recorded
    bool m_arapInProgress = false; // True when ARAP is running

    // Returns selection in a vector
    std::vector<int> getSelectedFaceIds();
    std::vector<int> getSelectedVertexIds();

    // Returns the mouse position
    Eigen::Vector2f getMousePosition();

    // Converts the camera position of the vertex to a world position
    Eigen::Vector3f convertCameraToWorldPosition(int);

    // Finds the closest vertex index to a selected face
    int findClosestVertexToSelection(int, const Eigen::Vector3f&) const;

    // Handles the selection (finds and stores the selected face and the closest vertex to the selection)
    bool handleSelection(igl::opengl::glfw::Viewer&);

    // Event listeners
    void handleKeyDownEvent();
    void handleMouseReleaseEvent();
    void handleMouseMoveEvent();
    void handleMouseDownEvent();
};

#endif //_GUI_H_
