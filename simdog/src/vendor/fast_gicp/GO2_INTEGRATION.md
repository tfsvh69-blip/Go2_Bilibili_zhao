# Go2 项目集成说明

本目录来自 `https://github.com/koide3/fast_gicp.git`，基准提交为：

```text
0e7ec1441c99f7be453db2ea216d5de029387417
```

上游许可证为 BSD 3-Clause，原始 `LICENSE` 已保留。为适配本项目的 Ubuntu
22.04、ROS 2 Humble、CUDA 12.8 和 RTX 5070，进行了以下本地调整：

- CUDA 架构通过 `FAST_GICP_CUDA_ARCHITECTURE` 配置，当前默认值为 `120`；统一
  构建脚本会优先读取目标 GPU 的 compute capability 并显式覆盖该值。
- CUDA 头文件直接包含新版 Thrust 类型定义，避免旧式前置声明在 CUDA 12.8
  中造成命名空间歧义。

嵌套 Git 元数据已移除以缩小项目体积；更新上游代码时应重新核对上述兼容改动。
