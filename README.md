## Week 3 PID 参数选择分析

### 1. 舵轮舵向电机的内环与外环

舵轮舵向电机采用串级控制：

```text
目标舵向角
    ↓
角度外环
    ↓
目标角速度
    ↓
速度内环
    ↓
目标力矩
    ↓
舵向电机
```
![舵轮舵向 PID 参数](./docs/week3/steering_pid.png)
舵向电机需要能够在各种条件下快速改变舵向，对响应速度的要求最大,且经常性的变化角度所有在外环使用单p控制输出目标速度，且这个p很大，响应速度理论上很快，同时稳态误差几乎可以忽略不计，所以不使用i项，同时外环产生目标角速度，如果使用d反而会在角度突变的时候引入微分冲击，导致震荡，内环将目标角速度转换为目标力矩输出，目标速度会随着角度误差的减小而变化，所以对稳态误差的感受不明显，反而追求响应速度，同样速度也是经常变化的，所以也不使用i和d，最终就变成使用较硬的p——p双环控制

### 2 摩擦轮

摩擦轮使用pd的速度环控制:
![摩擦轮 PID 参数](./docs/week3/friction_pid.png)

摩擦轮主要是快速升速、发弹掉速后快速恢复、不要产生太大的超调和振荡。理论上对精度有一定的要求，但实际上感觉是速度大加上直驱，转动惯量小，反而是震荡更明显，一点点的稳态误差在大的速度下效果可以忽略，所有会通过较大的p产生主要的力矩，再加入大量的d来抑制震荡，反倒是i在速度上升时容易达到积分限幅，减慢恢复速度

### 3 自瞄云台
自瞄时云台需要追踪动态目标，对精度和响应时间要求很高，所以烧饼为例，云台实际上大部分使用了pi：
![自瞄云台 PID 参数](./docs/week3/sentry_gimbal_pid.png)

这是一个经典的大小yaw的三轴云台，pitch轴和大yaw轴，都是使用双环pi控制，且整个云台的转动惯量大，稳态误差无法忽略，且会对自瞄精度产生较大的影响，所以使用了i项，小yaw轴比较特殊使用外环pid，内环p控制，因为小yaw轴只负责小幅度转动，转动惯量也不大更适合跟踪，控制精度的要求比较高，所以参数基本上比前面两个小。实际上因为i，又加入了很多的积分限幅防止积分过度累计产生震荡
![自瞄云台 PID 参数](./docs/week3/sentry_gimbal_limits.png)

但是pid依旧是有滞后性的，终究是要产生误差才能输出，而目标不会在动一下后等很久才动，要想打到就要提前预测，然后就加入了前馈控制
![自瞄云台 PID 参数](./docs/week3/sentry_gimbal_feedforward.png)

会对小yaw轴加入一个视觉的前馈来让它能够跟踪目标甚至预测目标的位置提前控制云台运动，大yaw和pitch都有较大的转动惯量所以震荡不明显，不用d，反而在内外环都加入了i项；而小yaw既有前馈有没多的转动惯量，所以加入了d减少震荡，而控制精度又要求很高，所以有i减少误差，内环由于阻力小就只用了p

但是最后pid使用什么还是要根据实际来，效果更好更稳定就使用，理论不完全和实际就相同。



# RMCS
RoboMaster Control System based on ROS2.

快速开始: [quick-start](https://github.com/Alliance-Algorithm/RMCS/wiki/Quick-Start)

## Development

### Pre-requirements:

- x86-64 架构
- 任意 Linux 发行版，或 WSL2（参见 [WSL2开发指南](docs/zh-cn/wsl2_develop_guide.md)）
- [VSCode](https://code.visualstudio.com/)，安装 [Dev Containers 扩展](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- [安装 Docker 并 配置代理（部分国家或地区）](docs/zh-cn/docker_with_proxy.md)

### Step 1：获取镜像

下载开发镜像：
```bash
docker pull qzhhhi/rmcs-develop:latest
```

如需交叉编译环境，可下载：
```bash
docker pull qzhhhi/rmcs-develop:latest-full
```

也可自行使用 `Dockerfile` 构建，参见 [镜像构建指南](docs/zh-cn/build_docker_image.md)。

### Step 2：克隆并打开仓库

克隆仓库，注意需要使用 `recurse-submodules` 以克隆子模块：

```bash
git clone --recurse-submodules https://github.com/Alliance-Algorithm/RMCS.git
```

在 VSCode 中打开仓库：

```bash
code ./RMCS
```

按 `Ctrl+Shift+P`，在弹出的菜单中选择 `Dev Containers: Reopen in Container`。

VSCode 将拉起一个 `Docker` 容器，容器中已配置好完整开发环境，之后所有工作将在容器内进行。

如果 `Dev Containers` 在启动时卡住很长一段时间，可以尝试 [这个解决方案](docs/zh-cn/fix_devcontainer_stuck.md)。

### Step 3：配置 VSCode

在 VSCode 中新建终端，输入：

```bash
cp .vscode/settings.default.json .vscode/settings.json
```

这会应用我们推荐的 VSCode 配置文件，你也可以按需自行修改配置文件。

在拓展列表中，可以看到我们推荐使用的拓展正在安装，你也可以按需自行删减拓展。

### Step 4：构建

在 VSCode 终端中输入：

```bash
build-rmcs
```

将会运行 `.script/build-rmcs` 脚本，在路径 `rmcs_ws` 下开始构建代码。

构建完毕后，基于 `clangd` 的 `C++` 代码提示将可用。此时可以正常编写代码。

Note: 用于开发的所有脚本均位于 `.script` 中，参见 开发脚本手册(TODO)。

如需在 `latest-full` 中执行交叉编译，请根据当前容器架构选择对向目标：
```bash
build-rmcs-cross --target-arch arm64
```
适用于 `linux/amd64` 的 `latest-full` 变体。

```bash
build-rmcs-cross --target-arch amd64
```
适用于 `linux/arm64` 的 `latest-full` 变体。

详见 [交叉编译使用说明](docs/zh-cn/cross_build.md)。

### Step 5 (Optional)：运行

编写代码并编译完成后，可以使用：

```bash
launch-rmcs
```

在本机上运行代码。在首次运行代码前，需要调用 `set-robot` 脚本设置机器人类型。

#### 确认设备接入

可以使用 `lsusb` 确定 [下位机](https://github.com/Alliance-Algorithm/rmcs_slave) 是否已接入，若已接入，则 `lsusb` 输出类似：

```
Bus 001 Device 004: ID a11c:75f3 Alliance RoboMaster Team. RMCS Slave v2.1.2
```

在 WSL2 下，需要 [使用 usbipd 对设备进行转接](docs/zh-cn/wsl2_develop_guide.md#step-5-optional)。

#### 确认权限正确

在主机（不要在 `docker` 容器）的终端中输入：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="a11c", MODE="0666"' | sudo tee /etc/udev/rules.d/95-rmcs-slave.rules &&
sudo udevadm control --reload-rules &&
sudo udevadm trigger
```

以允许非 root 用户读写 RMCS 下位机，此指令只需执行一次。

## Deployment

### Pre-requirements:

- x86-64 架构
- 任意 Linux 发行版
- [安装 Docker](docs/zh-cn/docker_with_proxy.md#ubuntu-安装-docker)

### Step 1：获取镜像

下载部署镜像：

```bash
docker pull qzhhhi/rmcs-develop:latest
```

如果不方便在 MiniPC 上配置代理，可以在开发机上下载镜像后，使用

```bash
docker save qzhhhi/rmcs-runtime:latest > rmcs-runtime.tar
```

然后使用任意方式（如 scp）将 `rmcs-runtime.tar` 传送到 MiniPC 上，并在其上执行：

```bash
docker load -i rmcs-runtime.tar
```

即获取部署镜像。

### Step 2：启动容器

在 MiniPC 终端中输入：

```bash
docker run -d --restart=always --privileged --network=host -v /dev:/dev qzhhhi/rmcs-runtime:latest
```

即可启动部署镜像，此后镜像将保持开机自启。

### Step 3：远程连接

在开发容器终端中输入：

```bash
set-remote <remote-host>
```

其中，`remote-host` 可以为 MiniPC 的：

1. IPv4 / IPv6 地址 (e.g., 169.254.233.233)

2. IPv4 / IPv6 Link-local 地址 (e.g., fe80::c6d0:e3ff:fed7:ed12%eth0)

3. mDNS 主机名 (e.g., my-sentry.local)

参见 网络配置指南(TODO)。

接下来在开发容器终端中继续输入：

```bash
ssh-remote
```

即可在开发容器中，ssh 连接到远程的部署容器。

**RMCS 的所有代码更新和调试，都基于从开发容器向部署容器的 ssh 连接。**

部署容器会监听 TCP:2022 端口作为 ssh-server 端口，请注意保证端口空闲。

参见 容器设计思想(TODO)。

> Tip: GUI 可以从部署容器中穿出，尝试在 ssh-remote 中打开 rviz2。

### Step 4：同步构建产物

新启动的部署容器内是没有代码的，需要由开发容器上传。

在开发容器中构建完成后，可以执行指令：

```bash
sync-remote
```

这将拉起一个同步进程，自动将开发容器中的构建产物同步到部署容器。

同步进程除非主动使用 `Ctrl+C` 结束，否则不会退出，其会监视所有文件变更，并实时同步到部署容器。

> Tip: 由于 `build-rmcs` 采用 `symlink-install` 方式构建，因此对于配置文件和 .py 文件，直接修改其源文件，无需编译即可触发同步。

### Step 5：重启服务

RMCS 在部署容器中以服务方式启动 (`/etc/init.d/rmcs`)。

确认构建产物同步完毕后（以出现 `Nothing to do` 为标志），进入 `ssh-remote`，输入：

```bash
set-robot <robot-name>
```

设置启动的机器人类型（例如 set-robot sentry）。

接下来继续输入：

```bash
service rmcs restart
```

如果一切正常，其将会输出：

```bash
Successfully stopped RMCS daemon.
Successfully started RMCS daemon.
```

这证明 RMCS 已成功在部署容器中启动，并会随以后的每次容器启动而启动。

接下来可以使用：

```bash
service rmcs attach
```

查看 RMCS 的实时输出。

> 需要注意的是，`service rmcs attach` 本质上是连接到了一个 `GNU screen` 会话，因此任何按键都会被忠实地转发至 RMCS。例如，当键入 `Ctrl+C` 时，RMCS 会接到 `SIGINT`，从而停止运行。
> 
> 如果希望退出对实时输出的查看，可以键入 `Ctrl+A` ，然后按 `D`。
> 
> 如果希望向上翻页，可以键入 `Ctrl+A` ，然后按 `Esc`。
> 
> 更多快捷键组合参见 [Screen Quick Reference](https://gist.github.com/andrimanna/e5379fe6db3af0ecdb1e49e8cfb74d24)。

### Step 6：糖

指令 `ssh-remote` 后可以添加参数，参数内容即为建立链接后立即执行的指令。

例如：

```bash
ssh-remote service rmcs restart
```

可以在开发容器端快速重启部署容器中的 RMCS。

又如：

```bash
ssh-remote service rmcs attach
```

可以在开发容器端快速查看部署容器中 RMCS 的实时输出。

实际上，可以使用指令：

```bash
attach-remote
```

作为前者的替代（其实现就是前者）。

进一步的，还可以使用：

```bash
attach-remote -r
```

作为 `ssh-remote "service rmcs restart && service rmcs attach"` 的替代。

其在重启 RMCS 守护进程后，自动连接显示实时输出。

更进一步的，指令间还可以组合，例如：

```bash
build-rmcs && wait-sync && attach-remote -r
```

可以触发 RMCS 构建，`wait-sync` 等待文件同步完成，接下来重启 RMCS 守护进程后，显示实时输出。
