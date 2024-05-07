#include <torch/torch.h>
#include <torch/script.h> // One-stop header.
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>


int main(int argc, char* argv[]) {
    struct timeval startTime, endTime;
    std::string model_path;
    
    auto qengines = at::globalContext().supportedQEngines();

if (std::find(qengines.begin(), qengines.end(), at::QEngine::QNNPACK) != qengines.end()) {

    at::globalContext().setQEngine(at::QEngine::QNNPACK);

}
    if (argc==1)
        model_path = "../model/model.pth";
    else
        model_path = argv[1];
    try {
        // 加载序列化的模型
        std::cout<<model_path<<std::endl;
        torch::jit::script::Module module;
        module = torch::jit::load(model_path);
        module.eval();
	torch::NoGradGuard no_grad;
        // 创建一个输入张量
        torch::Tensor tensorInput = torch::tensor({0.11412254, -0.09698317, -0.13648682, -0.08168529,  0.09584415,
        -0.08081015});
        tensorInput = tensorInput.reshape({1, 6});
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(tensorInput); // 假设输入是1x6大小
        // 进行预测
	    gettimeofday(&startTime, NULL);
        at::Tensor output = module.forward(inputs).toTensor();
	    gettimeofday(&endTime, NULL);
        float timeConsuming = (endTime.tv_sec - startTime.tv_sec)*1e3 + (endTime.tv_usec - startTime.tv_usec)*1e-3;
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
