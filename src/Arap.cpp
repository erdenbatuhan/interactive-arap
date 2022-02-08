/**
 * Project: Interactive ARAP
 * File:    SimpleArap.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#include "../include/Arap.h"

Arap::Arap() = default;

void Arap::updateParameters(const int movingVertex) {
    m_movingVertex = movingVertex;
}

std::vector<int> Arap::collectFixedVertices(Eigen::MatrixXd& faces, std::vector<int>& anchorFaces) const {
    std::vector<int> fixedVertices;

    // Add the vertices of each face to the fixed vertices
    for (int anchorFace : anchorFaces) {
        Eigen::VectorXi faceVertices = faces.row(anchorFace);

        for (int j = 0; j < faces.cols(); j++) {
            fixedVertices.push_back(faceVertices(j));
        }
    }

    // Add the selected vertex to the fixed vertices
    fixedVertices.push_back(m_movingVertex);
}

void Arap::runDeformation(Eigen::MatrixXd& vertices, Eigen::MatrixXd& faces, std::vector<int>& anchorFaceIds,
                          std::map<int, std::vector<int>>& neighborhood) const {
    std::vector<int> fixedVertices = collectFixedVertices(faces, anchorFaceIds);
}


int distBetRayPoint_anchor(glm::vec3 raynorm, glm::vec3 rayPoint, mesh3d thishelp, glm::mat4 globalmodel, float globalthresAnchor, float& min2ray) {
    glm::vec3 point1 = rayPoint;
    float y_sec = (-1 * rayPoint[0] * raynorm[1] / raynorm[0]) + rayPoint[1];
    float z_sec = (-1 * rayPoint[0] * raynorm[2] / raynorm[0]) + rayPoint[2];
    glm::vec3 point2 = glm::vec3(0.0, y_sec, z_sec);

    float thresho = globalthresAnchor;

    float min_disti = 1000.0;
    int min_disti_ind = 0.0;
    int secinter;
    for (int j = 0; j < thishelp.anchor_points.size();j++) {
        for (int i = 0; i < thishelp.anchor_points[j].size();i++) {
            glm::vec3 currtemp = thishelp.model_det[thishelp.anchor_points[j][i]].currVert;
            glm::vec4 new_cor = globalmodel * glm::vec4(currtemp, 1);
            float d = glm::length(glm::cross(point2 - point1, glm::vec3(new_cor[0], new_cor[1], new_cor[2]) - point1)) / glm::length(point2 - point1);
            if (d < min_disti) {
                min_disti = d;
                min_disti_ind = i;
                secinter = j;
            }

        }
    }

    return secinter;
}

glm::vec3 NN_Ray_Point(glm::vec3 rayPoint, glm::vec3 raynorm, glm::vec4 thisP) {
    glm::vec3 lineDir = glm::normalize(glm::vec3(raynorm[0], raynorm[1], raynorm[2]));
    glm::vec3 v = glm::vec3(thisP[0],thisP[1],thisP[2]) - rayPoint;
    float d = glm::dot(v, lineDir);
    return (rayPoint + (lineDir * d));
}

glm::vec3 clipCordinates(glm::vec3 inputivec) {
    for (int i = 0; i < 3;i++) {
        if (inputivec[i] <= -1.0) {
            inputivec[i] = -0.999;
        }
        else if (inputivec[i] >= 1.0) {
            inputivec[i] = 0.999;
        }
    }

    return inputivec;
}

mesh3d dragtoRay(glm::vec3 raynorm, glm::vec3 rayPoint, mesh3d thishelp, glm::mat4 globalmodel, float globalthresAnchor, mouseData gb_mouseData) {
    glm::vec3 point1 = rayPoint;
    float y_sec =  rayPoint[1]-( rayPoint[0] * raynorm[1] / raynorm[0]);
    float z_sec =  rayPoint[2] - ( rayPoint[0] * raynorm[2] / raynorm[0]);
    glm::vec3 point2 = glm::vec3(0.0, y_sec, z_sec);

    glm::mat4 invmod = glm::inverse(globalmodel);

    float min_dist = 100;

    std::vector<int> points_to_move;
    points_to_move.push_back(795);
    for (int i = 0; i < thishelp.model_det[796].allnei.size();i++) {
        points_to_move.push_back(thishelp.model_det[796].allnei[i]);
    }

    int break_out = 0;

    for (int i = 0; i < thishelp.anchor_points[gb_mouseData.select_anchor].size(); i++) {
        if (break_out > 100) {
            break;
        }

        break_out++;

        int currt = thishelp.anchor_points[gb_mouseData.select_anchor][i];
        thishelp.model_det[currt].currVert = clipCordinates(thishelp.model_det[currt].currVert);

        glm::vec4 currP = globalmodel * glm::vec4(thishelp.model_det[currt].currVert, 1);
        glm::vec3 cp1 = NN_Ray_Point(rayPoint, raynorm, currP);

        glm::vec3 diffi =  (cp1 - glm::vec3(currP[0],currP[1],currP[2]) );

        float scaler= 0.001;

        glm::vec3 lastPos1 = cp1;// +glm::vec3(scaler * diffi[0], scaler * diffi[1], scaler * diffi[2]);
        glm::vec3 lastPos3 = invmod * glm::vec4(lastPos1, 1);
        glm::vec3 lastPos2 = glm::vec3(lastPos3[0], lastPos3[1], lastPos3[2]);

        thishelp.model_det[currt].currVert = glm::vec3(lastPos2[0], lastPos2[1], lastPos2[2]);
        thishelp.verti[(currt * 6)] = lastPos2[0];
        thishelp.verti[(currt * 6) + 1] = lastPos2[1];
        thishelp.verti[(currt * 6) + 2] = lastPos2[2];
    }

    return thishelp;
}

mesh3d Arap_Initialize(mesh3d changeCor, mesh3d originalCor,glm::mat4 gb_model1,std::string mode,int curr_anchor) {
    Matrix4f newmat = Matrix4f::Identity();
    for (int x = 0; x < 4; x++) {
        for (int y = 0; y < 4; y++) {
            newmat(x, y) = gb_model1[x][y];
        }
    }
    Matrix4f newmat_inv = newmat.inverse();

    MatrixXf ccor= MatrixXf::Ones(originalCor.model_det.size(), 3);;
    MatrixXf ocor = MatrixXf::Ones(originalCor.model_det.size(),3);
    std::vector<std::vector<int>> nei_info;

    for (int i = 0; i < originalCor.model_det.size();i++) {
        ccor.row(i) = local2global(changeCor.model_det[i].currVert,gb_model1);
        ocor.row(i) = local2global(originalCor.model_det[i].currVert, gb_model1);
        nei_info.push_back(changeCor.model_det[i].allnei);
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    ccor = simple_arap(ccor,ocor,nei_info,originalCor.anchor_points[curr_anchor],originalCor.fixed_pts);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference  rotations = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[�s]" << std::endl;

    for (int i = 0;i< originalCor.model_det.size(); i++) {
        Vector4f final_resi1 = newmat_inv * Vector4f(ccor(i,0), ccor(i, 1), ccor(i, 2),1);
        glm::vec3 temp15 = glm::vec3(final_resi1[0], final_resi1[1], final_resi1[2]);

        changeCor.model_det[i].currVert = temp15;
        changeCor.verti[(i * 6)] = temp15[0];
        changeCor.verti[(i * 6) + 1] = temp15[1];
        changeCor.verti[(i * 6) + 2] = temp15[2];

    }

    return changeCor;
}

MatrixXf initialize_weights(MatrixXf originalCor, std::vector <std::vector<int>> nei_info) {
    MatrixXf weights_arap = MatrixXf::Ones(originalCor.rows(), originalCor.rows());

    bool debug_cotangent_weigts = FALSE;
    bool debug_uni = FALSE;

    if (debug_cotangent_weigts) {
        for (int i = 0; i < originalCor.rows(); i++) {
            std::cout << "computing cotangent weights" << std::endl;
        }
    } else if (debug_uni) {
        weights_arap = MatrixXf::Zero(originalCor.rows(), originalCor.rows());

        for (int i = 0; i < originalCor.rows(); i++) {
            for (int j = 0; j < nei_info[i].size(); j++) {
                weights_arap(i, nei_info[i][j]) = 1 / nei_info[i].size();

            }

            weights_arap(i, i) = 1.0;
        }
    }

    return weights_arap;
}

simp_arap update_RHS(simp_arap this_struct, MatrixXf changeCor, MatrixXf originalCor,
                     std::vector <std::vector<int>> nei_info, std::vector<int> debug_fixed_points, std::vector<int> to_move) {
    for (int i = 0; i < this_struct.tot_vert; i++) {
        Eigen::Vector3f temp10 = Eigen::Vector3f(0.0, 0.0, 0.0);
        //std::find(debug_fixed_points.begin(), debug_fixed_points.end(), i) != debug_fixed_points.end()
        //i < (this_struct.tot_vert - 1)

        if (std::find(debug_fixed_points.begin(), debug_fixed_points.end(), i) != debug_fixed_points.end()) {
            temp10 = originalCor.row(i);
            this_struct.LHS_LB_op.row(i).setZero();
            this_struct.LHS_LB_op(i, i) = 1;
        } else if (std::find(to_move.begin(), to_move.end(), i) != to_move.end()) {
            temp10 = changeCor.row(i);
            this_struct.LHS_LB_op.row(i).setZero();
            this_struct.LHS_LB_op(i, i) = 1;
        } else {
            for (int j = 0; j < nei_info[i].size(); j++) {
                Vector3f diff_edges = originalCor.row(i) - originalCor.row(nei_info[i][j]);
                Matrix3f adder = 0.5 * this_struct.weights(i, nei_info[i][j]) *
                                 (this_struct.all_rotations[i] + this_struct.all_rotations[nei_info[i][j]]);
                temp10 = temp10 + (adder * diff_edges);
            }
        }

        this_struct.RHS_b.row(i) = temp10;
    }

    return this_struct;
}

std::vector <Matrix3f> calculateRotations(simp_arap this_struct, MatrixXf changeCor, MatrixXf originalCor,
                                          std::vector <std::vector<int>> nei_info) {
    for (int i = 0; i < this_struct.tot_vert; i++) {
        Matrix3f curr_rotation = Matrix3f::Identity();

        int currTotNei = nei_info[i].size();
        MatrixXf plainP = MatrixXf::Zero(3, currTotNei);
        MatrixXf neighbourDiag = MatrixXf::Zero(currTotNei, currTotNei);
        MatrixXf shadP = MatrixXf::Zero(3, currTotNei);

        for (int j = 0; j < currTotNei; j++) {
            plainP.col(j) = originalCor.row(i) - originalCor.row(nei_info[i][j]);
            neighbourDiag(j, j) = this_struct.weights(i, nei_info[i][j]);
            shadP.col(j) = changeCor.row(i) - changeCor.row(nei_info[i][j]);;
        }

        Matrix3f final_mat = plainP * neighbourDiag * shadP.transpose();
        JacobiSVD <MatrixXf> svd(final_mat, ComputeThinU | ComputeThinV);
        curr_rotation = svd.matrixV() * svd.matrixU().transpose();

        if (curr_rotation.determinant() <= 0) {
            MatrixXf new_u = svd.matrixU();
            new_u.col(new_u.cols() - 1) = -1 * new_u.col(new_u.cols() - 1);
            curr_rotation = svd.matrixV() * new_u.transpose();
        }

        this_struct.all_rotations.push_back(curr_rotation);
    }

    return this_struct.all_rotations;
}

simp_arap update_LHS(simp_arap this_struct, std::vector <std::vector<int>> nei_info) {
    for (int i = 0; i < this_struct.tot_vert; i++) {
        for (int j = 0; j < nei_info[i].size(); j++) {
            this_struct.LHS_LB_op(i, nei_info[i][j]) = -1 * this_struct.weights(i, nei_info[i][j]);
            this_struct.LHS_LB_op(i, i) = this_struct.LHS_LB_op(i, i) + (this_struct.weights(i, nei_info[i][j]));
        }
    }

    return this_struct;
}

MatrixXf simple_arap(MatrixXf changeCor, MatrixXf originalCor, std::vector <std::vector<int>> nei_info, std::vector<int> to_move,
                     std::vector<int> fx_points) {
    simp_arap this_struct;

    int n_iterations = 1;
    this_struct.tot_vert = originalCor.rows();

    //initializing the weights
    this_struct.weights = initialize_weights(originalCor, nei_info);
    this_struct.RHS_b = MatrixXf::Zero(this_struct.tot_vert, 3);
    this_struct.LHS_LB_op = MatrixXf::Zero(this_struct.tot_vert, this_struct.tot_vert);
    this_struct = update_LHS(this_struct, nei_info);

    // the main optimization starts here
    for (int ni = 0; ni < n_iterations; ni++) {
        //calculate the rotations for each vertice
        this_struct.all_rotations.clear();
        this_struct.all_rotations = calculateRotations(this_struct, changeCor, originalCor, nei_info);

        //calculate the RHS
        this_struct = update_RHS(this_struct, changeCor, originalCor, nei_info, fx_points, to_move);

        SparseLU <SparseMatrix<float>> solver;
        solver.compute(this_struct.LHS_LB_op.sparseView());

        changeCor = solver.solve(this_struct.RHS_b);
    }

    return changeCor;
}
