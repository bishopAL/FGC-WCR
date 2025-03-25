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
    MatrixXd mat(rows, cols); //从文件中读取矩阵数据。
    ifstream file(filename);//从文件中读取向量数据。

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
} //从指定文件中读取矩阵数据，并将其存储在MatrixXd类型的变量中。

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
}//从指定文件中读取向量数据，并将其存储在VectorXd类型的变量中。

MatrixXd relu(const MatrixXd& x) {
    return x.cwiseMax(0.0);
}//实现ReLU激活函数，即将输入矩阵中的负值设为0

int main() {

    std::vector<float> stanceLegPositions(6); // 初始化向量，6个元素
    std::ifstream ifs("stance_leg_positions.bin", std::ios::binary);
    ifs.read(reinterpret_cast<char*>(stanceLegPositions.data()), stanceLegPositions.size() * sizeof(float));
    ifs.close();

    struct timeval startTime, endTime;
    // Load weights and biases
    int inputNum = 6;
    int layer1Num = 8;
    int outputNum = 3; //定义输入、隐藏层和输出的大小。
    MatrixXd W1 = load_matrix("../model/fc1.weight_weight.txt", layer1Num, inputNum);
    VectorXd b1 = load_vector("../model/fc1.bias_weight.txt", layer1Num);
    MatrixXd W2 = load_matrix("../model/fc2.weight_weight.txt", outputNum, layer1Num);
    VectorXd b2 = load_vector("../model/fc2.bias_weight.txt", outputNum);//从文件中加载权重和偏置。


    /////////////////////
    fnn(vector);
    // Example input
    VectorXd input(6);
    // input << 0.11412254, -0.09698317, -0.13648682, -0.08168529,  0.09584415,
    //     -0.08081015;  // Example values
     for (int i = 0; i < 6; ++i) {
        input(i) = stanceLegPositions[i];
    }
    VectorXd outputReal(3);
    outputReal << -0.01443261, -0.01519005, -0.02687549;//设置示例输入向量和预期输出向量。

    gettimeofday(&startTime, NULL);//记录开始时间。
    // Forward pass
    MatrixXd layer1 = relu((input.transpose() * W1.transpose()).transpose() + b1);//进行第一层的计算并应用ReLU激活函数。
    MatrixXd output = (layer1.transpose() * W2.transpose()).transpose() + b2;//计算输出层的结果
    gettimeofday(&endTime, NULL);
    float timeConsuming = (endTime.tv_sec - startTime.tv_sec)*1e3 + (endTime.tv_usec - startTime.tv_usec)*1e-3;//记录结束时间并计算时间消耗。

    std::ofstream ofs("/received_output.bin", std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(output.data()), output.size() * sizeof(float));
    ofs.close();

    cout << "Output:\n" << output << endl;

    std::cout <<"Real value: "<<  outputReal << std::endl;
    std::cout << "Time consuming: "<< timeConsuming << std::endl;//打印前向传播的输出结果、预期的实际输出值和时间消耗。

    return 0;
}
