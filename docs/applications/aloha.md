---
sidebar: auto
---
## 软件环境配置

> * 系统要求：Ubuntu20.04LTS
>
> * 根据实际需求配置相应的环境（如进行数据采集只需配置数据采集环境即可）。
>
> * 仓库clone权限（后续可能会用到）：
>
>   * User name: OpenGHz&#x20;
>
>   * Token (read only): glpat-YNYiRPBQHx395LANh8vJ

### 基本环境配置

* 安装git:

```bash
sudo apt install git -y
```

* 下载安装miniconda py38的最后一版:

```bash
wget https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/Miniconda3-py38_4.9.2-Linux-x86_64.sh
chmod u+x Miniconda3-py38_4.9.2-Linux-x86_64.sh
./Miniconda3-py38_4.9.2-Linux-x86_64.sh
```

### 数据采集环境

> 可以通过deb包或源码编译两种方式进行arm-control软件安装。若都安装，则将以最后一次安装的为准（请先完成[基本环节配置](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#PYxldPISIoZtF3xAUo9cUj59nTg)）。

#### Deb包安装

> 注意事项：
>
> * Deb包仅能支持数据采集，如果有其他使用需求，请使用[源码安装](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#Z66sdAB0hotbwQx8YqfcsHhWnnc)。
>
> * 若系统中有通过源码安装过旧版本的arm-control，可能会引起冲突，建议继续用[源码安装](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#Z66sdAB0hotbwQx8YqfcsHhWnnc)。

下载最新Deb包： #TODO: 等待测试后合并到develop分支然后直接下载develop即可。

下载完成后终端进入下载目录，然后进行安装：

```python
sudo apt update
sudo rm -rf /usr/local/include/airbot && sudo apt install ./airbot_play_2.6.2-44eac082_amd64.deb
```

#### 源码安装

##### 依赖配置

安装依赖（curses、spdlog、fmt）：

```python
sudo apt install libncurses5-dev libspdlog-dev -y
git clone --depth 1 https://github.com/fmtlib/fmt.git
cd fmt && mkdir build && cd build
cmake .. && make -j32 && sudo make install
```

##### 下载源码

```python
git clone --depth 1 https://git.qiuzhi.tech:20000/airbot-play/control/arm-control.git -b develop_ghz
```

修改.gitmodules中的https://git.qiuzhi.tech/airbot-play/arm-models为：https://git.qiuzhi.tech:20000/airbot-play/arm-models，然后开启梯子执行如下命令（若arm-models无法clone，则关闭梯子后再执行一次该命令）：

##### 编译安装

### 数据转换环境

> 需先配置[数据采集环境](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#MXnldmMNUoLeVJxS50lcW77Fnid)。

### 数据回放环境

> * 需先配置[数据转换环境](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#LlYvdcqgTozwCixv5uIcYk0FnXg)。
>
> * 该步配置也即机械臂Python控制接口配置。

下载whl文件后，cd到下载路径，安装：

### 模型训练环境

> 需先配置[数据转换环境](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#MCnydl3HEoio8hxtZAVch8MfnU5)。

### 模型推理环境

> 配置[数据回放环境](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#LlYvdcqgTozwCixv5uIcYk0FnXg)和[模型训练环境](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#P9ORdO7EGoRTRLxFJllctg8NnDb)即可。

### 测试数据样例

提供单双臂任务各三组原始数据用于测试环境是否安装正常。

## 机械臂使用说明

> 请先熟悉一下如何使用机械臂。
>
> 《用户手册》

### 面板介绍

机械臂面板实物图如下：

![](images/aloha-image-1.png)

* 红框部分是开/关机键，**长按3s左右待指示灯条全部亮起后完成开/关机**。

* 绿框部分是电源接口。

* 蓝框部分是通信接口。**注意不要连接到上图中的USB-1口**。

### 开机与标零

> 请先配置[数据采集环境](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#MXnldmMNUoLeVJxS50lcW77Fnid)。

* 通过电源适配器连接机械臂和电源，通过Type-C数据线连接机械臂与电脑（先连接示教臂，即人手操作的臂，再连接执行臂），然后长按开机键开机。

* 观察机械臂是否在零位。若不在零位或不知道是否在零位，请进行标零。可参考如下视频：

具体的标零命令如下（注意执行该命令后机械臂会受重力而下坠，请先扶好机械臂再执行该命令）：

* 示教臂标零（人手操作的那台臂）：

* 执行臂标零：

* 注意执行标零命令后**手动将机械臂末端夹爪推动至闭合位置**。

* 双臂操作时使用时只需要依次增加can序号完成上述步骤即可。

## 数据采集

> * 需先配置[数据采集环境](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#MXnldmMNUoLeVJxS50lcW77Fnid)。
>
> * 任务命名应合理，建议命名中带上时间，用于区分不同时间采集的同一任务数据。

### 启动机械臂

1. 准备两台带末端夹爪和相机的机械臂，一台机械臂用于示教（人手操作），另一台用于执行。

2. 连接两台机械臂的电源（顺序无要求）。

3. **先**通过USB连接示教臂（对应can0），**再**通过USB连接执行臂（对应can1）。

4. 长按开机。

5. 确保机械臂在零点，否则需要进行[标零](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#MV4SdrV2lohdvsxHW9Qc83Phnkg)。

### 连接相机

数据采集需要用到3个相机，其连接顺序有要求：

旧版：

* 单臂任务连接次序：臂上相机 -> 左侧（高侧）环境相机 -> 右侧（低侧）环境相机

* 双臂任务连接次序：左臂相机 -> 右臂相机 -> 左侧（高侧）环境相机 -> 右侧（低侧）环境相机

新版：

* 单臂任务连接次序：臂上相机左目 -> 臂上相机右目 -> 底座相机

* 双臂任务连接次序：TODO

### 运行采集程序

**参数含义说明：**

* `-c`：USB相机的设备号，多个设备号的顺序（即相机顺序）由具体任务指定

  * 一般按照连接顺序，可用的相机设备号为自增偶数，如`0`, `2`, `4`...

    * 若电脑自带普通摄像头，该摄像头占用一个设备号，外部连接相机设备号为从 `2` 开始的自增偶数：`2`, `4`, `6` ...

    * 若电脑自带带深度 / 结构光摄像头（如 Windows Hello 摄像头），RGB 相机和深度相机各自占用一个设备号，外部连接相机设备号为从 `4` 开始的自增偶数：`4`, `6`, `8` ...

* `-mts`：指定采集的最大帧数；由具体任务指定

  * 当达到最大时间步后，程序会进行提示

* `-tn`：任务名称，由具体任务指定

  * 决定了任务将保存到当前目录下的demonstrations/raw的哪个子文件夹里。

* `-se`：起始采集的episode序号，默认为0

  * 程序中断后可以修改该值以继续采集而不覆盖之前采集的任务

  * 误保存数据时可以指定该episode序号进行重新采集覆盖

* `-sjp`：每个episode开始采集前，各关节和夹爪的初始位置，不使用该参数时默认均为0；由具体任务指定

* `-f`：数据采集频率，默认为15Hz

**运行示例：**

**程序启动后的按键说明：**

不可连按 / 按住按键，否则按键的行为将全部顺序重复执行

* `g`：开启 / 关闭重力补偿

* 空格：开始 / 停止 episode 记录

* `q`：放弃本次记录

* `0`：机械臂回到初始位置

* `p`：打印当前关节位置信息

**操作步骤：**

1. 启动程序，出现 3 个相机的实时界面

   1. 如果无法启动 或 出现的界面数小于 3，联系&#x20;

2. 按**空格**，开始记录数据；同时开始**遥操作**机械臂完成任务

3. 任务完成后，等待采集到指定帧数（任务完成所用帧数应尽可能接近最大采集帧数）：

   1. 若不符合采集要求，按 `q` 舍弃当前示教记录，然后按`0`键控制机械臂回到初始位置。

   2. 若符合标准，按**空格**保存当前示教记录，记录保存完后，机械臂将自动返回初始位置。

4. （可选）在当前目录下的 `demonstrations/raw/<task_name>/` 文件夹中，检查记录的 episode

   * 采集到的每个 episode 数据包含：3 个相机录制的视频（`.avi`） 、1 个机械臂状态记录文件（`.json`）

**补充说明：**

1. 尽量保证完成任务的动作在**即将到达最大帧数前**完成，即不要过早结束动作。

2. 机械臂移动速度不可过快，否则采集的信息较稀疏，且图像的采集质量不高。

3. 将采集到的任务数据\<task\_name>文件夹存储到**指定硬盘**中的ALOHA/demonstrations/raw文件夹中备份。

## 数据转换

> 需先配置[数据转换环境](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#MCnydl3HEoio8hxtZAVch8MfnU5)。

首先保证激活conda环境并修改文件权限：

在**采集数据命令执行的相同路径**下，执行如下命令将数据保存至hdf5格式：

参数说明：

* -cn：指定相机的名称。

* -tn：指定任务名，**与数据采集时指定的任务名相同**。

* -se：指定数据的起始序号。

* -ee：指定数据的结束序号。

* -rn：双臂任务指定-rn 2。

该命令将从当前目录下的demonstrations/raw文件夹中寻找指定名称的任务数据，将其转换到demonstrations/hdf5文件夹中。

最后记得将采集到的任务数据\<task\_name>文件夹存储到**指定硬盘**中的ALOHA/demonstrations/hdf5文件夹中备份。

## 数据回放（可选）

> 通过数据回放的效果可以验证采集的数据、机械臂的零点设置等是否有问题（需先配置[数据回放环境](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#LlYvdcqgTozwCixv5uIcYk0FnXg)）。

先激活conda环境：

命令及参数说明如下：

* -tn：指定任务名称

* -ei：指定hdf文件对应的id

* -cb：初始是哪个can id

* -ii：不回放相机数据

* -ia：不回放动作数据

## 参数配置

**模型训练、推理前**需要先进行参数配置。在act/task\_configs文件中**创建一个与任务名相同的Python文件（不是在example\_task.文件中修改）**对任务进行配置，主要包括**修改各种路径**（通过replace\_task\_name使用**默认路径**或手动指定路径替换）、相机名称（camera\_names）和机器人数量（robot\_num，**双臂任务设置为2**）等等。以下是example\_task.py中的示例，其演示了如何基于template.py中的默认配置进行修改，而不需要全部重写（全部可调整配置详见act/task\_configs/template.py）：

![](images/aloha-image-2.png)

使用**默认路径训练**时，需要将转换得到的hdf5格式的数据放于`act/data/hdf5/<task_name>`文件夹中，可执行如下命令创建目录：

可执行如下命令进行复制：

> * CKPT\_DIR和STATS\_PATH若不存在将会**自动创建并写入**相关文件。

## 模型训练

> 请先完成[模型训练环境配置](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#P9ORdO7EGoRTRLxFJllctg8NnDb)和[参数配置](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#KchBdojIIo18u4xG7nicTym2nAf)（Please use at least 2 collected data to train）。

上述命令不带任何命令行参数时**将根据指定的任务名使用前述task\_configs中的配置**，若使用命令行参数，则将覆盖前述配置，这样可以方便临时更改参数，但并不推荐。

训练结束后，默认可以在./my\_ckpt/\<task\_name>目录下看到**两个文件夹**，其中ckpt中存储了中间过程的权重文件（称为**过程文件夹**）；而与\<task\_name>同名的文件夹（称为**核心文件夹**）下存储了最后权重、最优权重、统计数据、训练关键信息（包含初始关节角、训练参数配置等）以及训练过程的loss曲线图等文件：

![](images/aloha-image.png)

为了方便后续使用，将**核心文件夹**存储到**指定硬盘**中的ALOHA/my\_ckpt文件夹中。

> 训练调参建议：
>
> \- Chunk size is the most important param to tune when applying ACT to a new environment. One chunk should correspond to \~1 secs wall-clock robot motion. (原文50Hz控制频率用了90的chunk size)
>
> \- High KL weight (10 or 100), or train ***without*** CVAE encoder.
>
> \- Consider removing temporal\_agg and increase query frequency [here](https://github.com/tonyzhaozh/act/blob/main/imitate_episodes.py#L193) to be the same as your chunk size. I.e. each chunk is executed fully.
>
> \- train for **very long** (well after things plateaus, see picture)
>
> \- Try to increase batch size as much as possible, and increase lr accordingly. E.g. batch size 64 with learning rate 5e-5 versus batch size 8 and learning rate 1e-5
>
> \- Have separate backbones for each camera (requires changing the code, see [this commit](https://github.com/tonyzhaozh/act/commit/20fc6e990698534b89a41c2c85f54b7ff4b0c280))
>
> \- L1 loss > L2 loss (not precise enough)
>
> \- Abs position control > delta/velocity control (harder to recover)
>
> \- Try multiple checkpoints
>
>
>
> For real-world experiments:
>
> \- Train for even longer (5k - 8k epochs, especially if multi-camera)
>
> \- If inference is too slow -> robot moving slowly: disable temporal\_agg and increase query frequency [here](https://github.com/tonyzhaozh/act/blob/main/imitate_episodes.py#L193). We tried as high as 20.

## 模型使用

> 请先完成[模型推理环境配置](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#JW3XdhzIQojkoSxb31MclroFnPh)和[参数配置](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#KchBdojIIo18u4xG7nicTym2nAf)。

### 环境准备

* 首先将示教臂和执行臂的USB接口都拔掉以刷新CAN接口，然后只连接执行臂的USB接口（这样执行臂将使用CAN0）。

* 按照与数据采集时一样的顺序[连接相机](https://w79rvfxw83.feishu.cn/wiki/Uur1w5GB1ivNPVkyLYxcyXjFnjJ#OyGWdWWaVoEeykxY8JacJYw1n5d)（若采集完没有拔出过相机则跳过该步骤即可）。

* 长按开启机械臂电源开关。

### 执行命令

先进入act文件夹中并激活conda环境：

命令及参数说明如下：

* -ci: 相机设备号，需要与配置的相机名字的对应的设备顺序相对应。

* -ts: 任务对应的时间戳（查看模型训练结果保存的路径获取，例如：act/my\_ckpt/test\_task/20240325-153007）。若不采用默认路径配置可能没有时间戳，此时可移除该参数。

* -cki: 不启动机械臂，仅展示采集到的相机图像，用于确定相机顺序是否与采集数据时一致。

* -can: 指定使用哪个can进行控制，默认是can0；改为can1则：`-can can1`。对于双臂任务，需要指定多个can，如：`-can can0,can1`。

机械臂启动后会移动到任务定义的初始位置，等待终端提示，然后**按回车键开始进行推理**。推理结束按回车键结束，机械臂将返回零位。

每次评估结束后，将在eval\_results文件夹中看到该任务的评估结果相关文件（包含过程视频）。

## 信息查看

模型训练后在key\_info.pkl文件中存储了训练时的关键数据，可以通过如下方式查看。

首先进入act并激活conda环境：

然后执行如下命令查看指定时间戳（注意是训练结果保存的时间戳）的信息：

将在终端看到与该任务相关的信息输出，例如：

![](images/aloha-image-3.png)

其中包括了训练时的全部配置、训练数据的初始关节角等信息。通过这些信息，我们可以确定如下关键信息：

* 训练时使用的hdf5数据位于的绝对路径。

* 训练时的参数配置。

* 推理时应该使用的初始关节角。

* 其他信息。

这些信息为实验的可复现性提供保证。由于目前相机与机械臂固连，因此机械臂智能系统本身的复现较为容易。对于物体摆放位置则可以通过重放示教数据的方式确定。

