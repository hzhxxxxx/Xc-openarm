# 网线直连ROS2通信配置指南

## 一、硬件连接

1. 使用一根网线连接机器A和机器B
2. 机器A保持WiFi连接(正常上网)
3. 机器B可以断网

## 二、网络配置

### 机器A配置(本机)

1. 查看网络接口:
```bash
ip a
```
找到有线网口名称(如 `eno1`, `enp3s0` 等)

2. 配置静态IP:
```bash
sudo nmcli con add type ethernet ifname eno1 con-name direct-link ip4 192.168.137.1/24
```

3. 激活连接:
```bash
sudo nmcli con up direct-link
```

4. 验证配置:
```bash
ip a show eno1
```
应该看到 `inet 192.168.137.1/24`

### 机器B配置(远程机器)

1. 查看网络接口:
```bash
ip a
```
找到有线网口名称(如 `eno1`, `enp3s0` 等)

2. 配置静态IP:
```bash
sudo nmcli con add type ethernet ifname enp3s0 con-name direct-link ip4 192.168.137.2/24
```

3. 激活连接:
```bash
sudo nmcli con up direct-link
```

4. 验证配置:
```bash
ip a show enp3s0
```
应该看到 `inet 192.168.137.2/24`

### 测试网络连通性

从机器A ping机器B:
```bash
ping 192.168.137.2
```

从机器B ping机器A:
```bash
ping 192.168.137.1
```

## 三、ROS2配置

### 机器A配置

编辑 `~/.bashrc`:
```bash
nano ~/.bashrc
```

添加以下内容(如果已存在则修改):
```bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
```

使配置生效:
```bash
source ~/.bashrc
```

### 机器B配置

编辑 `~/.bashrc`:
```bash
nano ~/.bashrc
```

添加以下内容:
```bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
```

使配置生效:
```bash
source ~/.bashrc
```

**注意:** 两台机器的 `ROS_DOMAIN_ID` 必须相同!

## 四、测试ROS2通信

### 方法1: 简单话题测试

**机器A发布话题:**
```bash
ros2 topic pub /test std_msgs/msg/String "data: 'Hello from Machine A'"
```

**机器B查看话题列表:**
```bash
ros2 topic list
```
应该能看到 `/test` 话题

**机器B订阅话题:**
```bash
ros2 topic echo /test
```
应该能看到机器A发布的消息

### 方法2: demo节点测试

**机器A运行talker:**
```bash
ros2 run demo_nodes_cpp talker
```

**机器B运行listener:**
```bash
ros2 run demo_nodes_cpp listener
```

如果能看到消息接收,说明通信成功!

## 五、性能测试

### 测试发布频率
```bash
ros2 topic hz /test
```

### 测试带宽
```bash
ros2 topic bw /test
```

### 测试延迟
可以通过发布带时间戳的消息,对比发送和接收时间差。

## 六、常见问题

### 1. ping不通
- 检查网线是否插好
- 检查静态IP配置是否正确
- 检查防火墙设置

### 2. ROS话题看不到
- 确认两台机器的 `ROS_DOMAIN_ID` 相同
- 确认 `ROS_LOCALHOST_ONLY=0`
- 重新 `source ~/.bashrc`

### 3. 机器A上网受影响
- 检查是否配置了正确的网口(WiFi应保持独立)
- 有线口 `eno1/enp3s0` 只用于直连,WiFi口 `wlp4s0` 用于上网

## 七、优势

相比WiFi局域网:
- ✅ 延迟更低
- ✅ 更稳定
- ✅ 不依赖路由器
- ✅ 机器B无需联网
- ✅ 带宽独享
