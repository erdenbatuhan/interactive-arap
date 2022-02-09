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

    // Populate neighborhood: Neighborhood of vertices (Mapping between vertex id and its neighbor ids)
    populateNeighborhood();

    // Instantiate ARAP instance
    arap = new Arap();
}

Mesh::~Mesh() {
    delete arap;
}

void Mesh::populateNeighborhood() {
    for (int vertexId = 0; vertexId < m_vertices.rows(); vertexId++) {
        std::vector<int> allNeighbors;
        std::vector<int> distinctNeighbors;

        // Iterate over the edges
        for (int faceId = 0; faceId < m_faces.rows(); faceId++) {
            if (m_faces(faceId, 0) == vertexId) {
                allNeighbors.push_back(m_faces(faceId, 1));
                allNeighbors.push_back(m_faces(faceId, 2));
            }

            if (m_faces(faceId, 1) == vertexId) {
                allNeighbors.push_back(m_faces(faceId, 0));
                allNeighbors.push_back(m_faces(faceId, 2));
            }

            if (m_faces(faceId, 2) == vertexId) {
                allNeighbors.push_back(m_faces(faceId, 0));
                allNeighbors.push_back(m_faces(faceId, 1));
            }
        }

        for (auto neighbor: allNeighbors) {
            // Push to distinct neighbors if it is not in the list
            if (std::find(distinctNeighbors.begin(), distinctNeighbors.end(), neighbor) == distinctNeighbors.end()) {
                distinctNeighbors.push_back(neighbor);
            }
        }

        m_neighborhood[vertexId] = distinctNeighbors;
    }
}

Eigen::Vector2f Mesh::getMousePosition() const {
    return Eigen::Vector2f { m_viewer.current_mouse_x, m_viewer.core().viewport(3) - (float) m_viewer.current_mouse_y };
}

int Mesh::findClosestVertexToSelection(const int faceId, const Eigen::Vector3f& barycentricPosition) {
    // Vertex with the highest coefficient is the closest (P = wA + uB + vC)
    int maxCoefficient; barycentricPosition.maxCoeff(&maxCoefficient);
    return m_faces.row(faceId)(maxCoefficient);
}

Eigen::Vector3f Mesh::convertCameraToWorldPosition(int vertexId) const {
    Eigen::Vector2f mousePosition = getMousePosition();
    Eigen::Vector3f vertexPosition = {
        (float) m_vertices.row(vertexId).x(), (float) m_vertices.row(vertexId).y(), (float) m_vertices.row(vertexId).z()
    };

    Eigen::Vector3f projection = igl::project(vertexPosition, m_viewer.core().view, m_viewer.core().proj, m_viewer.core().viewport);
    Eigen::Vector3f worldPosition = igl::unproject(Eigen::Vector3f(mousePosition.x(), mousePosition.y(), projection.z()),
                                                   m_viewer.core().view, m_viewer.core().proj, m_viewer.core().viewport);

    return worldPosition;
}

bool Mesh::handleSelection(igl::opengl::glfw::Viewer& viewer, const bool toggleable = true) {
    int faceId;

    Eigen::Vector2f mousePosition = getMousePosition();
    Eigen::Vector3f barycentricPosition; // P = wA + uB + vC

    if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj, viewer.core().viewport,
                                 m_vertices, m_faces, faceId, barycentricPosition)) {
        if (m_arapInProgress) { // ARAP
            int movingVertex = findClosestVertexToSelection(faceId, barycentricPosition);
            arap->updateParameters(movingVertex, convertCameraToWorldPosition(movingVertex));
        } else { // Anchor point selection
            const bool selected = !toggleable || !m_anchorSelections[faceId];

            // Store the selections
            m_anchorSelections[faceId] = selected;

            // Set the color for the selected face
            m_colors.row(faceId) << 1, !selected, !selected;
            viewer.data().set_colors(m_colors);

            // Paint
            viewer.data().set_colors(m_colors);
        }

        return true;
    }

    return false;
}

void Mesh::handleMouseDownEvent() {
    m_viewer.callback_mouse_down = [this](igl::opengl::glfw::Viewer& viewer, int, int) -> bool {
        const bool selectionHandled = handleSelection(viewer);

        m_mouseDownBeingRecorded = selectionHandled;
        return selectionHandled;
    };
}

void Mesh::handleMouseReleaseEvent() {
    m_viewer.callback_mouse_up = [this](igl::opengl::glfw::Viewer& viewer, int, int) -> bool {
        m_mouseDownBeingRecorded = false;
        return true;
    };
}

bool Mesh::computeDeformation(igl::opengl::glfw::Viewer& viewer) {
    // Extract selected faces to a list
    std::vector<int> selectedFaceIds;
    for (auto entry : m_anchorSelections) {
        if (entry.second) { // If face is selected
            selectedFaceIds.push_back(entry.first);
        }
    }

    // Compute deformation
    Eigen::MatrixXd deformedVertices = arap->computeDeformation(m_vertices, m_faces, m_neighborhood, selectedFaceIds);
    m_vertices = safeReplicate(deformedVertices);

    viewer.data().compute_normals();
    viewer.data().set_mesh(m_vertices, m_faces);

    return true;
}

void Mesh::handleMouseMoveEvent() {
    m_viewer.callback_mouse_move = [this](igl::opengl::glfw::Viewer& viewer, int, int) -> bool {
        if (m_mouseDownBeingRecorded) {
            if (m_arapInProgress) { // Run ARAP deformation
                return computeDeformation(viewer);
            }

            // Selecting anchor points
            return handleSelection(viewer, false);
        }

        return false;
    };
}

void Mesh::handleKeyDownEvent() {
    m_viewer.callback_key_down = [this](igl::opengl::glfw::Viewer& viewer, unsigned char keyPressed, int) -> bool {
        if (keyPressed == 'A') { // Lock user input for ARAP
            m_arapInProgress = !m_arapInProgress; // Toggle ARAP

            std::vector<int> selectedFaceIds;
            for (const auto& entry : m_anchorSelections) {
                if (entry.second) {
                    selectedFaceIds.push_back(entry.first);
                    m_colors.row(entry.first) << !m_arapInProgress, m_arapInProgress, 0; // R = Selection, G = ARAP
                }
            }

            viewer.data().set_colors(m_colors);
            return true;
        } else if (keyPressed == 'R') { // Reset selections
            m_arapInProgress = false; // Stop ARAP

            // Remove the selections stored
            m_anchorSelections.clear();

            // Remove all the paints
            m_colors = Eigen::MatrixXd::Constant(m_faces.rows(), 3, 1);
            m_viewer.data().set_colors(m_colors);

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

    // Launch the glfw viewer
    m_viewer.launch();
}

