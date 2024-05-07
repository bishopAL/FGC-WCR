from scipy.io import loadmat
data = loadmat('trainingData.mat')
data = data['dataRecord']
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset

# 假设你的数据是numpy数组形式，需要转换成torch Tensor
import numpy as np

# 示例数据，你应该用你的实际数据替换这里的np.random.rand()
inputs = torch.tensor(data[:, :6], dtype=torch.float32)
outputs = torch.tensor(data[:, 6:], dtype=torch.float32)

# 创建数据集和数据加载器
dataset = TensorDataset(inputs, outputs)
dataloader = DataLoader(dataset, batch_size=10, shuffle=True)

# 定义神经网络模型
class SimpleFNN(nn.Module):
    def __init__(self):
        super(SimpleFNN, self).__init__()
        self.fc1 = nn.Linear(6, 6)  # 输入层到隐藏层，50个神经元
        self.fc2 = nn.Linear(6, 3)  # 输出层

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        return self.fc2(x)

model = SimpleFNN()

# 定义损失函数和优化器
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# 训练网络
epochs = 200
for epoch in range(epochs):
    for inputs, labels in dataloader:
        optimizer.zero_grad()
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()
    print(f'Epoch {epoch+1}, Loss: {loss.item()}')


# 预测
test_input = torch.tensor([[0.11412254, -0.09698317, -0.13648682, -0.08168529,  0.09584415,
        -0.08081015]], dtype=torch.float32)  # 示例输入数据
with torch.no_grad():
    prediction = model(test_input)
    print(f'Prediction: {prediction.numpy()}')
    print("-0.01443261, -0.01519005, -0.02687549")

# save the model
scripted_model = torch.jit.script(model)
torch.jit.save(scripted_model, 'model.pth')

data = loadmat('testingData.mat')
test_data = data['dataRecord']

# 测试数据集，应该与训练数据集具有相同的结构
test_inputs = torch.tensor(test_data[:, :6], dtype=torch.float32)
test_outputs = torch.tensor(test_data[:, 6:], dtype=torch.float32)

# 创建测试 DataLoader
test_dataset = TensorDataset(test_inputs, test_outputs)
test_loader = DataLoader(test_dataset, batch_size=10)

# 将模型设置为评估模式
model.eval()

# 初始化损失计算
test_loss = 0.0
with torch.no_grad():  # 确保不进行梯度计算
    for inputs, labels in test_loader:
        # 前向传播
        outputs = model(inputs)
        # 计算损失
        loss = criterion(outputs, labels)
        test_loss += loss.item()

# 平均损失
test_loss /= len(test_loader)
print(f'Average loss on the test dataset: {test_loss}')

