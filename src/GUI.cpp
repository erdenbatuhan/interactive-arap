/**
 * Project: Interactive ARAP
 * File:    GUI.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#include "../include/GUI.h"

GUI::GUI(const std::string& modelName) {
    // Load a mesh in OFF format
    igl::readOFF(modelName, m_vertices, m_faces);

    // Initialize white colors
    m_colors = Eigen::MatrixXd::Constant(m_faces.rows(), 3, 1);
}

std::vector<int> GUI::getSelectedFaceIds() const {
    std::vector<int> selectedFaceIds;

    for (const auto& entry : m_selectedAnchorVertexId) {
        selectedFaceIds.push_back(entry.first);
    }

    return selectedFaceIds;
}

std::vector<Vertex> GUI::getSelectedVertices() const {
    std::vector<Vertex> selectedVertexIds;

    for (const auto& entry : m_selectedAnchorVertexId) {
        selectedVertexIds.push_back(entry.second);
    }

    return selectedVertexIds;
}

Eigen::Vector2f GUI::getMousePosition() const {
    return Eigen::Vector2f { m_viewer.current_mouse_x, m_viewer.core().viewport(3) - (float) m_viewer.current_mouse_y };
}

int GUI::findClosestVertexIdToSelection(const int faceId, const Eigen::Vector3f& barycentricPosition) {
    // Vertex with the highest coefficient is the closest (P = wA + uB + vC)
    int maxCoefficient; barycentricPosition.maxCoeff(&maxCoefficient);
    return m_faces.row(faceId)(maxCoefficient);
}

std::vector<int> GUI::findNeighborIds(const int vertexId) const {
    std::vector<int> allNeighbors;
    std::vector<int> distinctNeighbors;

    // Iterate over the edges
    for (int i = 0; i < m_faces.rows(); i++) {
        if (m_faces(i, 0) == vertexId) {
            allNeighbors.push_back(m_faces(i, 1));
            allNeighbors.push_back(m_faces(i, 2));
        }

        if (m_faces(i, 1) == vertexId) {
            allNeighbors.push_back(m_faces(i, 0));
            allNeighbors.push_back(m_faces(i, 2));
        }

        if (m_faces(i, 2) == vertexId) {
            allNeighbors.push_back(m_faces(i, 0));
            allNeighbors.push_back(m_faces(i, 1));
        }
    }

    for (auto neighbor : allNeighbors) {
        // Push to distinct neighbors if it is not in the list
        if (std::find(distinctNeighbors.begin(), distinctNeighbors.end(), neighbor) == distinctNeighbors.end()) {
            distinctNeighbors.push_back(neighbor);
        }
    }

    return distinctNeighbors;
}

Eigen::Vector3f GUI::convertCameraToWorldPosition(int vertexId) const {
    Eigen::Vector2f position = getMousePosition();

    Eigen::Vector3f projection = igl::project((Eigen::Vector3f) m_vertices.row(vertexId).cast<float>(),
                                              m_viewer.core().view, m_viewer.core().proj, m_viewer.core().viewport);
    Eigen::Vector3f worldPosition = igl::unproject(Eigen::Vector3f(position.x(), position.y(), (float) projection.z()),
                                                   m_viewer.core().view, m_viewer.core().proj, m_viewer.core().viewport);

    return worldPosition;
}

bool GUI::handleSelection(igl::opengl::glfw::Viewer& viewer) {
    int faceId;

    Eigen::Vector2f mousePosition = getMousePosition();
    Eigen::Vector3f barycentricPosition; // P = wA + uB + vC

    if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj, viewer.core().viewport,
                                 m_vertices, m_faces, faceId, barycentricPosition)) {
        const int closestVertexIdToSelection = findClosestVertexIdToSelection(faceId, barycentricPosition);

        if (m_arapInProgress) { // Running the ARAP
            m_currentAnchorVertexId = closestVertexIdToSelection;
        } else if (m_selectedAnchorVertexId[faceId].id == INVALID_VERTEX_ID) { // Selecting the anchor point if it is not selected
            // Store the selections (each selected face stores the closest vertex to the selection)
            m_selectedAnchorVertexId[faceId] = Vertex {
                closestVertexIdToSelection, findNeighborIds(closestVertexIdToSelection)
            };

            // Set the color for the selected face
            m_colors.row(faceId) << 1, 0, 0;
            viewer.data().set_colors(m_colors);

            // Paint
            viewer.data().set_colors(m_colors);
        }

        return true;
    }

    return false;
}

void GUI::handleMouseDownEvent() {
    m_viewer.callback_mouse_down = [this](igl::opengl::glfw::Viewer& viewer, int, int) -> bool {
        const bool selectionHandled = handleSelection(viewer);

        m_mouseDownBeingRecorded = selectionHandled;
        return selectionHandled;
    };
}

void GUI::handleMouseReleaseEvent() {
    m_viewer.callback_mouse_up = [this](igl::opengl::glfw::Viewer& viewer, int, int) -> bool {
        m_mouseDownBeingRecorded = false;
        return true;
    };
}

void GUI::handleMouseMoveEvent() {
    m_viewer.callback_mouse_move = [this](igl::opengl::glfw::Viewer& viewer, int, int) -> bool {
        if (m_mouseDownBeingRecorded) {
            if (m_arapInProgress) { // Running ARAP
                std::cout << m_currentAnchorVertexId << std::endl;
                return true;
            }

            // Selecting anchor points
            return handleSelection(viewer);
        }

        return false;
    };
}

void GUI::handleKeyDownEvent() {
    m_viewer.callback_key_down = [this](igl::opengl::glfw::Viewer& viewer, unsigned char keyPressed, int) -> bool {
        if (keyPressed == 'A') { // Toggle ARAP
            m_arapInProgress = !m_arapInProgress;
            std::vector<int> selectedFaceIds = getSelectedFaceIds();

            // Visualize the selection toggle for selected faces (Red when selecting, Green when performing arap)
            for (int& selectedFaceId : selectedFaceIds) {
                m_colors.row(selectedFaceId) << !m_arapInProgress, m_arapInProgress, 0;
            }

            viewer.data().set_colors(m_colors);
            return true;
        } else if (keyPressed == 'R') { // Reset selections
            m_arapInProgress = false; // Stop ARAP

            // Remove the selections stored
            m_selectedAnchorVertexId.clear();

            // Remove all the paints
            m_colors = Eigen::MatrixXd::Constant(m_faces.rows(), 3, 1);
            m_viewer.data().set_colors(m_colors);

            return true;
        }

        m_arapInProgress = false;
        return false;
    };
}

void GUI::launchViewer() {
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
