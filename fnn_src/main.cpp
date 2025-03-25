#include <torch/torch.h> //引入LibTorch库以处理模型加载和推理。
#include <torch/script.h> // One-stop header.
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

//这段代码使用了PyTorch的C++接口（LibTorch）来加载和运行一个序列化的神经网络模型，并测量推理过程的时间。代码的主要流程如下：

int main(int argc, char* argv[]) {
    struct timeval startTime, endTime;
    std::string model_path;//用于存储模型文件的路径
    
    auto qengines = at::globalContext().supportedQEngines();

if (std::find(qengines.begin(), qengines.end(), at::QEngine::QNNPACK) != qengines.end()) {

    at::globalContext().setQEngine(at::QEngine::QNNPACK);

}//检查是否支持QNNPACK量化引擎，如果支持则设置QNNPACK为当前量化引擎。
    if (argc==1)
        model_path = "../model/model.pth";
    else
        model_path = argv[1];//根据命令行参数获取模型文件的路径，如果没有提供参数则使用默认路径"../model/model.pth"。
    try {
        // 加载序列化的模型
        std::cout<<model_path<<std::endl;
        torch::jit::script::Module module; 
        module = torch::jit::load(model_path);//使用torch::jit::load函数加载序列化的模型文件，并将模型设置为评估模式。
        module.eval();
	torch::NoGradGuard no_grad; //使用torch::NoGradGuard禁用梯度计算。
        // 创建一个输入张量
        torch::Tensor tensorInput = torch::tensor({0.11412254, -0.09698317, -0.13648682, -0.08168529,  0.09584415,
        -0.08081015});
        tensorInput = tensorInput.reshape({1, 6});
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(tensorInput); // 将输入张量添加到inputs向量中，准备进行推理。
     
	    gettimeofday(&startTime, NULL);
        at::Tensor output = module.forward(inputs).toTensor();
	    gettimeofday(&endTime, NULL);
        float timeConsuming = (endTime.tv_sec - startTime.tv_sec)*1e3 + (endTime.tv_usec - startTime.tv_usec)*1e-3; //记录推理结束时间，并计算时间消耗（以毫秒为单位）。
        std::cout <<"Infer value: "<< output << std::endl;
        torch::Tensor outputReal = torch::tensor({-0.01443261, -0.01519005, -0.02687549});
        std::cout <<"Real value: "<<  outputReal << std::endl;
        std::cout << "Time consuming: "<< timeConsuming << std::endl;
    } catch (const c10::Error& e) {
        std::cerr << "error loading the model\n";
        return -1;
    }
    std::cout << "Model loaded and inference executed successfully\n";
    return 0;
}

//这段代码的主要目的是加载一个PyTorch模型文件，进行前向传播推理，并测量其时间消耗。这对于模型性能测试和优化非常有用。
