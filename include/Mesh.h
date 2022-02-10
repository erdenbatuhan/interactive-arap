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

#include "Eigen.h"
#include "Arap.h"

#include <map>
#include <vector>

#define INVALID_VERTEX -1

class Mesh {
public:
    explicit Mesh(const std::string&);
    ~Mesh() = default;

    // Launches the GLFW viewer
    void launchViewer();
private:
    // Vertices, colors and faces of the model
    Eigen::MatrixXd m_vertices{}, m_colors{};
    Eigen::MatrixXi m_faces{};

    // GLFW viewer
    igl::opengl::glfw::Viewer m_viewer{};

    // ARAP instance
    Arap m_arap;

    // Selections stored
    int m_movingVertex = INVALID_VERTEX; // Selected moving vertex to be used to perform ARAP
    std::map<int, bool> m_anchorSelections; // Selected faces (anchors)

    // State variables that is keeping track of the state
    bool m_selectionHandledOnMesh = false; // If clicked on a point on the mesh
    bool m_arapInProgress = false; // If ARAP is running

    // Returns the mouse position
    static Eigen::Vector2f getMousePosition(igl::opengl::glfw::Viewer&);

    // Finds the closest vertex index to a selected face
    int findClosestVertexToSelection(int, const Eigen::Vector3f&);

    // Converts the camera position of the vertex to a world position
    Eigen::Vector3f convertCameraToWorldPosition(igl::opengl::glfw::Viewer&, int) const;

    // Computes the ARAP deformation
    void computeDeformation(igl::opengl::glfw::Viewer&);

    // Handles the selection (finds and stores the selected face and the closest vertex to the selection)
    bool handleSelection(igl::opengl::glfw::Viewer&, bool);

    // Event listeners
    void handleKeyDownEvent();
    void handleMouseReleaseEvent();
    void handleMouseMoveEvent();
    void handleMouseDownEvent();
};

#endif // _MESH_H_

