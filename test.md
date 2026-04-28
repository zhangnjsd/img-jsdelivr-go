# 卡尔曼滤波和虚拟串口

## 卡尔曼滤波

卡尔曼滤波是一种递归算法，用于估计动态系统的状态。它基于线性高斯状态空间模型，通过结合测量数据和系统模型来提供最小均方误差意义下的最优状态估计。卡尔曼滤波器由两个主要步骤组成：预测和更新。

### 预测步骤

在预测步骤中，卡尔曼滤波器使用系统模型来预测当前状态的先验估计。这个步骤包括以下公式：

$$
\hat{x}_{k|k-1} = A \hat{x}_{k-1|k-1} + B u_k
$$
$$
P_{k|k-1} = A P_{k-1|k-1} A^T + Q
$$

其中，$\hat{x}_{k|k-1}$ 是当前状态的先验估计，$P_{k|k-1}$ 是先验估计的协方差矩阵，$A$ 是状态转移矩阵，$B$ 是控制输入矩阵，$u_k$ 是控制输入，$Q$ 是过程噪声协方差矩阵。

> 下面通过一个**一维直线运动的小车跟踪**例子，来演示卡尔曼滤波的预测步骤是如何工作的。
>
> **场景设定**
> 我们要追踪一辆沿直线运动的小车。状态向量包含**位置**和**速度**：
> \[
> x = \begin{bmatrix} p \\ v \end{bmatrix}
> \]
> 时间间隔设为 \(\Delta t = 1\) 秒。假设小车近似做匀速运动，但速度可能存在微小随机波动（过程噪声）。本例中**没有任何控制输入**（油门未知），因此 \(B u_k = 0\)。
>
> ---
>
> **步骤 1：确定模型矩阵**
>
> **状态转移矩阵**（匀速模型）：
> \[
> A = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} 1 & 1 \\ 0 & 1 \end{bmatrix}
> \]
> **过程噪声协方差矩阵** \(Q\)：用来刻画速度的随机波动（加速度噪声）。设加速度噪声方差 \(\sigma^2_a = 0.1\ (m/s^2)^2\)，离散化后：
> \[
> Q = \sigma^2_a \begin{bmatrix} \frac{\Delta t^4}{4} & \frac{\Delta t^3}{2} \\ \frac{\Delta t^3}{2} & \Delta t^2 \end{bmatrix} = 0.1 \times \begin{bmatrix} 0.25 & 0.5 \\ 0.5 & 1 \end{bmatrix} = \begin{bmatrix} 0.025 & 0.05 \\ 0.05 & 0.1 \end{bmatrix}
> \]
>
> ---
>
> **步骤 2：上一时刻 \(k-1\) 的后验估计**
> 假设在 \(k-1\) 时刻，我们经过测量更新后，得到：
>
> - 后验状态估计：\(\hat{x}_{k-1|k-1} = \begin{bmatrix} 2.0 \\ 3.0 \end{bmatrix}\) （位置 2 m，速度 3 m/s）
> - 后验协方差矩阵：\(P_{k-1|k-1} = \begin{bmatrix} 1.0 & 0 \\ 0 & 1.0 \end{bmatrix}\)（对位置和速度的估计都较确定）
>
> ---
>
> **步骤 3：执行预测**
> **状态预测**
>
> \[
> \hat{x}_{k|k-1} = A \hat{x}_{k-1|k-1} + B u_k 
> = \begin{bmatrix} 1 & 1 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} 2.0 \\ 3.0 \end{bmatrix} + 0 
> = \begin{bmatrix} 2.0 + 3.0 \\ 3.0 \end{bmatrix} 
> = \begin{bmatrix} 5.0 \\ 3.0 \end{bmatrix}
> \]
>
> - 预测位置变为 5 m（按匀速运动从 2 m 走过 3 m），速度保持 3 m/s。
>
> **协方差预测**
>
> \[
> P_{k|k-1} = A P_{k-1|k-1} A^T + Q
> \]
> 先计算 \(A P_{k-1|k-1} A^T\)，因为 \(P_{k-1|k-1} = I\)：
> \[
> A I A^T = \begin{bmatrix} 1 & 1 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} 1 & 0 \\ 1 & 1 \end{bmatrix} = \begin{bmatrix} 2 & 1 \\ 1 & 1 \end{bmatrix}
> \]
> 加上过程噪声 \(Q\)：
> \[
> P_{k|k-1} = \begin{bmatrix} 2 & 1 \\ 1 & 1 \end{bmatrix} + \begin{bmatrix} 0.025 & 0.05 \\ 0.05 & 0.1 \end{bmatrix} = \begin{bmatrix} 2.025 & 1.05 \\ 1.05 & 1.1 \end{bmatrix}
> \]
>
> ---
>
> **步骤 4：结果解读**
>
> - **预测状态** \(\hat{x}_{k|k-1} = [5.0, 3.0]^T\) 就是模型在无新测量时，对当前时刻小车位置和速度的最优猜测。  
> - **预测协方差** \(P_{k|k-1}\) 变大了（对角线元素从 1 增加到 2.025 和 1.1），这反映了预测过程中由于过程噪声 \(Q\) 引入的额外不确定性——仅凭运动模型推导，我们对位置、速度的把握会越来越“散”。
>
> ---
>
> **补充：如果有控制输入 \(u_k\)**
> 假设小车有一个已知的恒定加速度 \(a = 1\ m/s^2\)，可作为控制输入 \(u_k = a\)。此时控制输入矩阵 \(B\) 为：
> \[
> B = \begin{bmatrix} \frac{1}{2}\Delta t^2 \\ \Delta t \end{bmatrix} = \begin{bmatrix} 0.5 \\ 1 \end{bmatrix}, \quad B u_k = \begin{bmatrix} 0.5 \\ 1 \end{bmatrix}
> \]
> 预测状态将变成：
> \[
> \hat{x}_{k|k-1} = A\hat{x}_{k-1|k-1} + B u_k = \begin{bmatrix} 5.0 \\ 3.0 \end{bmatrix} + \begin{bmatrix} 0.5 \\ 1.0 \end{bmatrix} = \begin{bmatrix} 5.5 \\ 4.0 \end{bmatrix}
> \]
> 协方差预测公式不变，因为控制输入是确定量，不增加不确定性。
>
> 这就是卡尔曼滤波预测步骤的完整数值示例。

### 更新步骤

在更新步骤中，卡尔曼滤波器使用测量数据来更新状态估计。这个步骤包括以下公式：

$$
y_k = z_k - H \hat{x}_{k|k-1}
$$

$$
S_k = H P_{k|k-1} H^T + R
$$

$$
K_k = P_{k|k-1} H^T S_k^{-1}
$$

$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k y_k
$$

$$
P_{k|k} = (I - K_k H) P_{k|k-1}
$$

其中，$y_k$ 是测量残差，$S_k$ 是残差协方差矩阵，$K_k$ 是卡尔曼增益，$\hat{x}_{k|k}$ 是当前状态的后验估计，$P_{k|k}$ 是后验估计的协方差矩阵，$H$ 是测量矩阵，$R$ 是测量噪声协方差矩阵。这里给出的 $P_{k|k}$ 是常见的简化写法；在数值实现中，也常用更稳定的 Joseph 形式。

> 接续前面的小车追踪例子，我们继续用**更新步骤**把测量数据融合进来，修正预测的状态。
>
> ---
>
> **场景承接**
>
> - 时刻 \(k\) 的**先验估计**（仅靠匀速模型推得）：  
>   \[
>   \hat{x}_{k|k-1} = \begin{bmatrix} 5.0 \\ 3.0 \end{bmatrix},\quad
>   P_{k|k-1} = \begin{bmatrix} 2.025 & 1.05 \\ 1.05 & 1.1 \end{bmatrix}
>   \]
>
> **测量设置**
> 小车上装有 **GPS 定位**，只能测量**位置**，测量值带噪声。  
>
> - 测量矩阵：\( H = \begin{bmatrix} 1 & 0 \end{bmatrix} \)  
> - 测量噪声协方差（位置测量方差）：假设 \(R = 0.5\) (米²)  
> - 当前时刻的真实测量值：\( z_k = 5.8 \) 米（即 GPS 读到 5.8 m）
>
> ---
>
> **更新步骤计算**
>
> 1. 测量残差 \(y_k\)（创新）
> \[
> y_k = z_k - H \hat{x}_{k|k-1}
>      = 5.8 - \begin{bmatrix} 1 & 0 \end{bmatrix} \begin{bmatrix} 5.0 \\ 3.0 \end{bmatrix}
>      = 5.8 - 5.0 = 0.8 \text{ 米}
> \]
> 含义：实际测量比模型预测的位置多了 0.8 m，这个差值将驱动状态修正。
>
> ---
>
> 2. 残差协方差 \(S_k\)（创新协方差）
> \[
> S_k = H P_{k|k-1} H^T + R
> \]
> 先算 \(H P_{k|k-1} H^T\)：  
> \[
> H P_{k|k-1} = \begin{bmatrix} 1 & 0 \end{bmatrix}
> \begin{bmatrix} 2.025 & 1.05 \\ 1.05 & 1.1 \end{bmatrix}
> = \begin{bmatrix} 2.025 & 1.05 \end{bmatrix}
> \]
> 再右乘 \(H^T\)：  
> \[
> (H P_{k|k-1}) H^T = \begin{bmatrix} 2.025 & 1.05 \end{bmatrix}
> \begin{bmatrix} 1 \\ 0 \end{bmatrix} = 2.025
> \]
> 加上 \(R = 0.5\)：  
> \[
> S_k = 2.025 + 0.5 = 2.525
> \]
> 含义：预测的位置观测值（映射到测量空间）的方差，加上测量噪声方差，就是**残差的总不确定性**。这里 \(S_k\) 是一个标量，因为测量是一维的。
>
> ---
>
> 3. 卡尔曼增益 \(K_k\)
> \[
> K_k = P_{k|k-1} H^T S_k^{-1}
> \]
> 计算 \(P_{k|k-1} H^T\)：  
> \[
> P_{k|k-1} H^T = \begin{bmatrix} 2.025 & 1.05 \\ 1.05 & 1.1 \end{bmatrix}
> \begin{bmatrix} 1 \\ 0 \end{bmatrix}
> = \begin{bmatrix} 2.025 \\ 1.05 \end{bmatrix}
> \]
> 除以 \(S_k = 2.525\)：  
> \[
> K_k = \frac{1}{2.525} \begin{bmatrix} 2.025 \\ 1.05 \end{bmatrix}
>      \approx \begin{bmatrix} 0.8020 \\ 0.4158 \end{bmatrix}
> \]
> 含义：
>
> - 第一行 0.8020 是“位置修正系数”——测量残差 0.8 m 中的 80.2% 会被用于修正位置。  
> - 第二行 0.4158 表示测量位置信息也间接修正了速度估计，因为本例中速度不可直接观测，但通过位置残差和状态协方差的相关性（\(P_{k|k-1}\) 非对角项）得到修正。  
> 卡尔曼增益自动权衡了**模型预测的不确定性**（\(P_{k|k-1}\)）与**测量噪声**（\(R\)）。
>
> ---
>
> 4. 状态更新（后验估计 \(\hat{x}_{k|k}\)）
> \[
> \hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k y_k
> \]
> \[
> \hat{x}_{k|k} = \begin{bmatrix} 5.0 \\ 3.0 \end{bmatrix} + \begin{bmatrix} 0.8020 \\ 0.4158 \end{bmatrix} \times 0.8 = \begin{bmatrix} 5.0 \\ 3.0 \end{bmatrix} + \begin{bmatrix} 0.6416 \\ 0.3326 \end{bmatrix} = \begin{bmatrix} 5.6416 \\ 3.3326 \end{bmatrix}
> \]
> 解释：
>
> - 测量值 5.8 m 把位置从预设的 5.0 m 向上拉了 0.6416 m，得到 5.6416 m（既不完全信模型，也不完全信测量，加权折衷）。  
> - 速度也从 3.0 m/s 修正到了 3.3326 m/s，因为位置测出的增量暗示了之前速度可能偏低。
>
> ---
>
> 5. 协方差更新（后验协方差 \(P_{k|k}\)）
> 使用简化形式：
> \[
> P_{k|k} = (I - K_k H) P_{k|k-1}
> \]
> \[I - K_k H = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix} - \begin{bmatrix} 0.8020 \\ 0.4158 \end{bmatrix} \begin{bmatrix} 1 & 0 \end{bmatrix} = \begin{bmatrix} 1 - 0.8020 & 0 \\ -0.4158 & 1 \end{bmatrix} = \begin{bmatrix} 0.1980 & 0 \\ -0.4158 & 1 \end{bmatrix} \]
> 乘以 \(P_{k|k-1}\)：
> \[
> P_{k|k} \approx \begin{bmatrix} 0.1980 & 0 \\ -0.4158 & 1 \end{bmatrix}
> \begin{bmatrix} 2.025 & 1.05 \\ 1.05 & 1.1 \end{bmatrix}
> = \begin{bmatrix} 
> 0.1980\times 2.025 + 0 & 0.1980\times 1.05 + 0 \\
> (-0.4158)\times 2.025 + 1\times 1.05 & (-0.4158)\times 1.05 + 1\times 1.1
> \end{bmatrix}
> \]
> 计算：
>
> - \(P_{11} = 0.1980 \times 2.025 \approx 0.4010\)
> - \(P_{12} = 0.1980 \times 1.05 \approx 0.2079\)
> - \(P_{21} = -0.8420 + 1.05 = 0.2080\) （约等于 \(P_{12}\)，对称）
> - \(P_{22} = -0.4366 + 1.1 = 0.6634\)
>
> 所以：
> \[
> P_{k|k} \approx \begin{bmatrix} 0.4010 & 0.2079 \\ 0.2080 & 0.6634 \end{bmatrix}
> \]
> 对比预测协方差 \(P_{k|k-1} = \begin{bmatrix} 2.025 & 1.05 \\ 1.05 & 1.1 \end{bmatrix}\)，后验方差大幅减小：  
>
> - 位置方差从 2.025 降至 0.4010（因测量提供了直接位置信息）。
> - 速度方差从 1.1 降至 0.6634（通过相关性也降低了不确定性）。  
> 整个状态的不确定性被测量有效“压缩”了。
>
> ---
>
> **补充：数值稳定的 Joseph 形式**
> 上面用了简化式 \(P_{k|k} = (I - K_k H) P_{k|k-1}\)。当卡尔曼增益计算有舍入误差时，此式可能破坏协方差的对称正定性。  
> **Joseph 形式**能保证结果对称正定：
> \[
> P_{k|k} = (I - K_k H) P_{k|k-1} (I - K_k H)^T + K_k R K_k^T
> \]
> 在实际工程中，尤其是状态维数高、计算矩阵求逆时，常使用 Joseph 形式以提高数值稳健性。本例中如果代入计算，会得到和简化结果非常接近的对称矩阵（误差主要由小数取舍引起），但数学性质更安全。
>
> ---
>
> 这样，我们就完整走完了从**预测**到**更新**的一个滤波周期。滤波器现在有了时刻 \(k\) 的最优状态估计 \(\hat{x}_{k|k} = [5.6416, 3.3326]^T\) 及其协方差，可以继续进入下一轮预测。

## 虚拟串口

虚拟串口是一种软件模拟的串行通信接口，允许应用程序通过标准串行通信协议进行数据传输，而无需实际的物理串口设备。虚拟串口通常用于调试、测试和模拟串行通信环境。

### 创建虚拟串口

#### Windows

在Windows系统中，可以使用第三方软件如 Virtual Serial Port Driver 来创建虚拟串口。

此处使用 [com0com](https://github.com/vovsoft/com0com/tree/main) 作为示例，安装后可以创建一对虚拟串口，例如 COM8 和 COM9。应用程序可以通过 COM8 进行写操作，而另一个应用程序可以通过 COM9 进行读操作，从而实现数据传输。

![虚拟串口示例](https://cdn.jsdelivr.net/gh/zhangnjsd/img-jsdelivr-go/vSerial/w1n.png)

`Apply` 后会生成一对虚拟串口，分别为 COM8 和 COM9。

![虚拟串口示例](https://cdn.jsdelivr.net/gh/zhangnjsd/img-jsdelivr-go/vSerial/w2.png)

#### Linux

在Linux系统中，通常使用伪终端（PTY）来模拟串口，并由工具生成对应的设备节点。例如，可以使用以下命令创建一对虚拟串口：

```bash
sudo socat -d -d PTY,link=/dev/ttyV0,raw,echo=0 PTY,link=/dev/ttyV1,raw,echo=0
```

![虚拟串口示例](https://cdn.jsdelivr.net/gh/zhangnjsd/img-jsdelivr-go/vSerial/1.png)

### 使用虚拟串口

#### Windows

我在串口监视器中连接两个串口，分别为 COM8 和 COM9，在 COM8 中输入数据后，COM9 中会显示相应的数据输出。

![虚拟串口通信示例](https://cdn.jsdelivr.net/gh/zhangnjsd/img-jsdelivr-go/vSerial/w3.png)

#### Linux

创建虚拟串口后，应用程序可以通过 `/dev/ttyV0` 和 `/dev/ttyV1` 进行通信。一个应用程序可以打开 `/dev/ttyV0` 进行写操作，而另一个应用程序可以打开 `/dev/ttyV1` 进行读操作，从而实现数据传输。虚拟串口在调试和测试串行通信协议时非常有用，可以模拟各种通信场景，而无需实际的硬件设备。

创建完成后在两个不同的终端中分别使用输入输出来操作虚拟端口。

`ttyV0` 为写端(TX)，`ttyV1` 为读端(RX)时，在一个终端中输入数据后，另一个终端会显示相应的数据输出。

![虚拟串口通信示例](https://cdn.jsdelivr.net/gh/zhangnjsd/img-jsdelivr-go/vSerial/2.png)
![虚拟串口通信示例](https://cdn.jsdelivr.net/gh/zhangnjsd/img-jsdelivr-go/vSerial/3.png)

> 其实二者没有严格的读写端之分，数据在两个端口之间是双向传输的。无论哪个端口进行写操作，另一个端口都可以接收数据。
