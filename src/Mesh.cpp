/**
 * Project: Interactive ARAP
 * File:    GUI.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#include "../include/Mesh.h"

Mesh::Mesh(const std::string& modelName) {
    // Load a mesh in OFF format
    igl::readOFF(modelName, m_vertices, m_faces);

    // Initialize white colors
    m_colors = Eigen::MatrixXd::Constant(m_faces.rows(), 3, 1);
}

Eigen::Vector2f Mesh::getMousePosition(igl::opengl::glfw::Viewer& viewer) {
    return Eigen::Vector2f { viewer.current_mouse_x, viewer.core().viewport(3) - (float) viewer.current_mouse_y };
}

int Mesh::findClosestVertexToSelection(const int faceId, const Eigen::Vector3f& barycentricPosition) {
    // Vertex with the highest coefficient is the closest (P = wA + uB + vC)
    int maxCoefficient; barycentricPosition.maxCoeff(&maxCoefficient);
    return m_faces.row(faceId)(maxCoefficient);
}

bool Mesh::handleSelection(igl::opengl::glfw::Viewer& viewer, const bool toggleable = true) {
    int faceId;

    Eigen::Vector2f mousePosition = getMousePosition(viewer);
    Eigen::Vector3f barycentricPosition; // P = wA + uB + vC

    if (m_arapInProgress) { // ARAP
        if (m_movingVertex == INVALID_VERTEX) { // If no moving vertex has been selected yet
            if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj, viewer.core().viewport,
                                         m_vertices, m_faces, faceId, barycentricPosition)) {
                // Find the closest vertex to selection
                m_movingVertex = findClosestVertexToSelection(faceId, barycentricPosition);

                // Instantiate ARAP
                m_arap.precomputeDeformation(m_vertices, m_faces);

                return true;
            }
        }
    } else { // Anchor point selection
        if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj, viewer.core().viewport,
                                     m_vertices, m_faces, faceId, barycentricPosition)) {
            const bool selected = !toggleable || !m_anchorSelections[faceId];

            // Store the selections
            m_anchorSelections[faceId] = selected;

            // Set the color for the selected face
            m_colors.row(faceId) << 1, !selected, !selected;
            viewer.data().set_colors(m_colors);

            // Paint
            viewer.data().set_colors(m_colors);
            return true;
        }
    }

    return false;
}

void Mesh::handleMouseDownEvent() {
    m_viewer.callback_mouse_down = [this](igl::opengl::glfw::Viewer& viewer, int, int) -> bool {
        const bool selectionHandledOnMesh = handleSelection(viewer);

        m_selectionHandledOnMesh = selectionHandledOnMesh;
        return selectionHandledOnMesh;
    };
}

void Mesh::handleMouseReleaseEvent() {
    m_viewer.callback_mouse_up = [this](igl::opengl::glfw::Viewer& viewer, int, int) -> bool {
        m_selectionHandledOnMesh = false;
        m_movingVertex = INVALID_VERTEX; // Reset the selection of moving vertex

        return true;
    };
}

Eigen::Vector3f Mesh::convertCameraToWorldPosition(igl::opengl::glfw::Viewer& viewer, int vertexId) const {
    Eigen::Vector2f mousePosition = getMousePosition(viewer);
    Eigen::Vector3f vertexPosition = {
        (float) m_vertices.row(vertexId).x(), (float) m_vertices.row(vertexId).y(), (float) m_vertices.row(vertexId).z()
    };

    Eigen::Vector3f projection = igl::project(vertexPosition, viewer.core().view, viewer.core().proj, viewer.core().viewport);
    Eigen::Vector3f worldPosition = igl::unproject(Eigen::Vector3f(mousePosition.x(), mousePosition.y(), projection.z()),
                                                   viewer.core().view, viewer.core().proj, viewer.core().viewport);

    return worldPosition;
}

void Mesh::computeDeformation(igl::opengl::glfw::Viewer& viewer) {
    // Extract selected faces to a list
    std::vector<int> selectedFaces;
    for (auto entry : m_anchorSelections) {
        if (entry.second) { // If face is selected
            selectedFaces.push_back(entry.first);
        }
    }

    // Compute the updated position of moving vertex
    m_arap.updateMovingVertex(m_movingVertex, convertCameraToWorldPosition(viewer, m_movingVertex),
                              m_faces, selectedFaces);

    // Compute deformation
    Eigen::MatrixXd deformedVertices = m_arap.computeDeformation(m_vertices);
    m_vertices = safeReplicate(deformedVertices);

    viewer.data().compute_normals();
    viewer.data().set_mesh(m_vertices, m_faces);
}

void Mesh::handleMouseMoveEvent() {
    m_viewer.callback_mouse_move = [this](igl::opengl::glfw::Viewer& viewer, int, int) -> bool {
        if (m_selectionHandledOnMesh) {
            handleSelection(viewer,  false); // Selecting anchor points

            if (m_arapInProgress) {
                computeDeformation(viewer); // Deformation
            }

            return true;
        }

        return false;
    };
}

void Mesh::handleKeyDownEvent() {
    m_viewer.callback_key_down = [this](igl::opengl::glfw::Viewer& viewer, unsigned char keyPressed, int) -> bool {
        if (keyPressed == 'A') { // Activate ARAP: Locks any user input other than moving vertex selection
            m_arapInProgress = !m_arapInProgress; // Toggle ARAP

            // Collect all selected face ids in a list
            std::vector<int> selectedFaceIds;
            selectedFaceIds.reserve(m_anchorSelections.size());

            for (const auto& entry : m_anchorSelections) {
                if (entry.second) {
                    selectedFaceIds.push_back(entry.first);
                    m_colors.row(entry.first) << !m_arapInProgress, m_arapInProgress, 0; // R = Selection, G = ARAP
                }
            }

            // Paint
            viewer.data().set_colors(m_colors);
            return true;
        } else if (keyPressed == 'R') { // Reset selections
            m_arapInProgress = false; // Stop ARAP

            // Remove the selections stored
            m_anchorSelections.clear();

            // Remove all the paints
            m_colors = Eigen::MatrixXd::Constant(m_faces.rows(), 3, 1);
            viewer.data().set_colors(m_colors);

            return true;
        }

        m_arapInProgress = false;
        return false;
    };
}

void Mesh::launchViewer() {
    // Call event handlers
    handleMouseDownEvent();
    handleMouseReleaseEvent();
    handleMouseMoveEvent();
    handleKeyDownEvent();

    // Plot the mesh
    m_viewer.data().set_mesh(m_vertices, m_faces);
    m_viewer.data().set_colors(m_colors);

    // Disable wireframe
    m_viewer.data().show_lines = false;

    // Launch the glfw viewer
    m_viewer.launch();
}

