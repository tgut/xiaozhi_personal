#pragma once
#include <string>
#include <vector>
#include <functional>

class BleFileTransfer {
public:
    // 初始化BLE服务，设置回调
    void Init(std::function<void(bool)> on_transfer_done);
    // 发送文件（分包）
    void SendFile(const std::string& filepath);
    // 检查是否已连接
    bool IsConnected() const;
};
