/**
 * Project: Interactive ARAP
 * File:    Mesh.h
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#ifndef _MESH_H_
#define _MESH_H_

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>

#include <map>
#include <vector>

#define INVALID_VERTEX_ID -1

class Mesh {
public:
    explicit Mesh(const std::string&);

    // Launches the GLFW viewer
    void launchViewer();
private:
    // Vertices, colors and faces of the model
    Eigen::MatrixXd m_vertices{}, m_colors{};
    Eigen::MatrixXi m_faces{};

    // GLFW viewer
    igl::opengl::glfw::Viewer m_viewer{};

    // Neighborhood of vertices (Mapping between vertex id and its neighbor ids)
    std::map<int, std::vector<int>> m_neighborhood;

    // Selections stored
    std::map<int, bool> m_anchorSelections; // Selected faces (anchors)
    int m_movingVertexId = INVALID_VERTEX_ID; // The closest vertex to the latest selection is stored for ARAP

    // State variables that is keeping track of the state
    bool m_mouseDownBeingRecorded = false; // True when mouse down event is being recorded
    bool m_arapInProgress = false; // True when ARAP is running

    // Populates the neighborhood
    void populateNeighborhood();

    // Returns the mouse position
    Eigen::Vector2f getMousePosition() const;

    // Finds the closest vertex index to a selected face
    int findClosestVertexIdToSelection(int, const Eigen::Vector3f&);

    // Converts the camera position of the vertex to a world position
    Eigen::Vector3f convertCameraToWorldPosition(int) const;

    // Handles the selection (finds and stores the selected face and the closest vertex to the selection)
    bool handleSelection(igl::opengl::glfw::Viewer&, bool);

    // Event listeners
    void handleKeyDownEvent();
    void handleMouseReleaseEvent();
    void handleMouseMoveEvent();
    void handleMouseDownEvent();
};

#endif //_MESH_H_
