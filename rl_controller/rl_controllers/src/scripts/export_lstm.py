import torch

# 加载第一个 JIT 模型
model1 = torch.jit.load("lstm_encoder.jit")
model1.eval()

# 加载第二个 JIT 模型
# 假设你有另一个模型文件 policy_mlp.jit
model2 = torch.jit.load("policy_mlp.jit")
model2.eval()

# 创建第一个模型的示例输入（根据模型需求调整大小和类型）
example_input1 = torch.randn(1, 1, 45)  # 假设这是适合第一个模型的输入
hidden = torch.zeros(3, 1, 256)
cell = torch.zeros(3, 1, 256)

# 创建第二个模型的示例输入（根据模型需求调整大小和类型）
example_input2 = torch.randn(1, 69)  # 假设这是适合第二个模型的输入，调整维度以符合该模型

# 导出第一个模型
torch.onnx.export(model1, 
                  (example_input1, hidden ,cell),       # 第一个 JIT 模型
                  "lstm_encoder.onnx",  # 第一个模型的输出文件名
                  export_params=True,   # 导出模型的参数
                  opset_version=11,     # ONNX opset 版本
                  do_constant_folding=True,  # 优化常量折叠
                  input_names=['input', 'hidden_init', 'cell_init'],    # 输入名
                  output_names=['output', 'hidden_final', 'cell_final'],  # 输出名
                  )

# 导出第二个模型
torch.onnx.export(model2,               # 第二个 JIT 模型
                  example_input2,       # 第二个模型的示例输入
                  "policy_lstm.onnx",    # 第二个模型的输出文件名
                  export_params=True,   # 导出模型的参数
                  opset_version=11,     # ONNX opset 版本
                  do_constant_folding=True,  # 优化常量折叠
                  input_names=['input'],    # 输入名
                  output_names=['output'],  # 输出名
                  )
