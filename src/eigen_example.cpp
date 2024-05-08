#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

using namespace Eigen;
using namespace std;

MatrixXd load_matrix(const string& filename, int rows, int cols) {
    MatrixXd mat(rows, cols);
    ifstream file(filename);

    if (file.is_open()) {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                file >> mat(i, j);
            }
        }
        file.close();
    } else {
        cout << "Unable to open file";
    }
    return mat;
}

VectorXd load_vector(const string& filename, int size) {
    VectorXd vec(size);
    ifstream file(filename);

    if (file.is_open()) {
        for (int i = 0; i < size; ++i) {
            file >> vec(i);
        }
        file.close();
    } else {
        cout << "Unable to open file";
    }
    return vec;
}

MatrixXd relu(const MatrixXd& x) {
    return x.cwiseMax(0.0);
}

int main() {
    struct timeval startTime, endTime;
    // Load weights and biases
    MatrixXd W1 = load_matrix("../model/fc1.weight_weight.txt", 6, 6);
    VectorXd b1 = load_vector("../model/fc1.bias_weight.txt", 6);
    MatrixXd W2 = load_matrix("../model/fc2.weight_weight.txt", 3, 6);
    VectorXd b2 = load_vector("../model/fc2.bias_weight.txt", 3);

    // Example input
    VectorXd input(6);
    input << 0.11412254, -0.09698317, -0.13648682, -0.08168529,  0.09584415,
        -0.08081015;  // Example values
    VectorXd outputReal(3);
    outputReal << -0.01443261, -0.01519005, -0.02687549;

    gettimeofday(&startTime, NULL);
    // Forward pass
    MatrixXd layer1 = relu((input.transpose() * W1.transpose()).transpose() + b1);
    MatrixXd output = (layer1.transpose() * W2.transpose()).transpose() + b2;
    gettimeofday(&endTime, NULL);
    float timeConsuming = (endTime.tv_sec - startTime.tv_sec)*1e3 + (endTime.tv_usec - startTime.tv_usec)*1e-3;

    cout << "Output:\n" << output << endl;

    std::cout <<"Real value: "<<  outputReal << std::endl;
    std::cout << "Time consuming: "<< timeConsuming << std::endl;

    return 0;
}
