/**
 * Project: Interactive ARAP
 * File:    GUI.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 *
 * Reference repositories:
 *  - https://github.com/libigl/libigl/blob/main/tutorial/708_Picking/main.cpp
 */

#include "../include/GUI.h"

GUI::GUI(const std::string& modelName) {
    // Load a mesh in OFF format
    igl::readOFF(modelName, m_vertices, m_faces);

    // Initialize white colors
    m_colors = Eigen::MatrixXd::Constant(m_faces.rows(), 3, 1);
}

Eigen::Vector2f GUI::getMousePosition() {
    return Eigen::Vector2f { m_viewer.current_mouse_x, m_viewer.core().viewport(3) - (float) m_viewer.current_mouse_y };
}

Eigen::Vector3f GUI::convertCameraToWorldPosition(int vertexId) {
    Eigen::Vector2f position = getMousePosition();

    Eigen::Vector3f projection = igl::project((Eigen::Vector3f) m_vertices.row(vertexId).cast<float>(),
                                              m_viewer.core().view, m_viewer.core().proj, m_viewer.core().viewport);
    Eigen::Vector3f worldPosition = igl::unproject(Eigen::Vector3f(position.x(), position.y(), (float) projection.z()),
                                                   m_viewer.core().view, m_viewer.core().proj, m_viewer.core().viewport);

    return worldPosition;
}

void GUI::handleMouseClick() {
    m_viewer.callback_mouse_down = [this](igl::opengl::glfw::Viewer&, int, int) -> bool {
        int faceId;

        Eigen::Vector2f mousePosition = getMousePosition();
        Eigen::Vector3f barycentricPosition; // P = wA + uB + vC

        if (igl::unproject_onto_mesh(mousePosition, m_viewer.core().view, m_viewer.core().proj, m_viewer.core().viewport,
                                     m_vertices, m_faces, faceId, barycentricPosition)) {
            // Paint the face
            m_colors.row(faceId) << 1, 0, 0;
            m_viewer.data().set_colors(m_colors);

            // Vertex with the highest coefficient is the closest (P = wA + uB + vC)
            int maxCoefficient; barycentricPosition.maxCoeff(&maxCoefficient);
            m_anchorVertexIds.push_back(m_faces.row(faceId)(maxCoefficient));

            return true;
        }

        return false;
    };
}

void GUI::launchViewer() {
    // Call event handlers
    handleMouseClick();

    // Plot the mesh
    m_viewer.data().set_mesh(m_vertices, m_faces);
    m_viewer.data().set_colors(m_colors);

    // Launch the glfw viewer
    m_viewer.launch();
}
