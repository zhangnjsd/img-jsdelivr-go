# 卡尔曼滤波和虚拟串口

## 卡尔曼滤波

卡尔曼滤波是一种递归算法，用于估计动态系统的状态。它基于线性高斯状态空间模型，通过结合测量数据和系统模型来提供最小均方误差意义下的最优状态估计。卡尔曼滤波器由两个主要步骤组成：预测和更新。

以一维平衡数据为例演示滤波过程。

一个简单的状态空间模型可以表示为：

```rust
#[derive(Clone, Copy)]
struct LinearKalmanFilter {
    x: f64,     // 状态估计
    p: f64,     // 估计误差协方差
    k: f64,     // 卡尔曼增益
}
```

在预测步骤中，滤波器根据系统模型预测下一个状态和误差协方差：

```rust
fn update(mut self, measurement: f64, process_noise: f64, measurement_noise: f64) -> Self {
    // 预测
    let x_pred = self.x; // 线性模型假设状态不变
    let p_pred = self.p + process_noise;

    // 更新
    self.k = p_pred / (p_pred + measurement_noise);

    // 融合测量
    self.x = x_pred + self.k * (measurement - x_pred);

    // 更新误差协方差
    self.p = (1.0 - self.k) * p_pred;

    self
}
```

预定一群测量数据，使用卡尔曼滤波器进行状态估计：

```rust
let linear_data = vec![
    99.842, 100.123, 99.987, 
    100.456, 99.654, 99.995, 
    100.321, 99.876, 100.234, 
    99.789, 98.996, 99.256, 
    101.024, 99.654, 100.789,
];
```

对这个数据进行滤波处理：

```rust
let mut kf = LinearKalmanFilter::new(100.0, 1.0);

// 定义过程噪声（模型不确定性）和测量噪声（传感器误差） 
let (process_noise, measurement_noise) = (0.01, 0.2);

let filtered_data: Vec<f64> = linear_data.iter().map(|&measurement| {
    kf = kf.update(measurement, process_noise, measurement_noise);
    println!("测量: {:.3}, 估计: {:.3}, 卡尔曼增益: {:.3}", measurement, kf.x, kf.k);
    kf.x
}).collect();
```

可以看到滤波后数据的估计值趋于稳定，卡尔曼增益逐渐减小，说明滤波器对测量数据的依赖减少，状态估计变得更加可靠。

```text
原始数据: [99.842, 100.123, 99.987, 100.456, 99.654, 99.995, 100.321, 99.876, 100.234, 99.789, 98.996, 99.256, 101.024, 99.654, 100.789]
滤波后的数据: [99.86811570247934, 99.98776211357159, 99.98750158368266, 100.11939920704798, 100.00352142563962, 100.00156006079166, 100.07147042067359, 100.03005286090946, 100.07237918428537, 100.01434483277002, 99.80756337018062, 99.69617440759905, 99.96339520787498, 99.90126950762748, 100.07926826078872]
```

绘图后能更直观地看出数据的变化趋势：

![滤波结果示例](https://cdn.jsdelivr.net/gh/zhangnjsd/img-jsdelivr-go/kalman/k1.png)

> 图中橙色为源数据，紫色为滤波后的数据，可以看到滤波后的数据更平滑。
>
完整代码：

```rust
#[derive(Clone, Copy)]
struct LinearKalmanFilter {
    x: f64,     // 状态估计
    p: f64,     // 估计误差协方差
    k: f64,     // 卡尔曼增益
}

impl LinearKalmanFilter {
    fn new(initial_x: f64, initial_p: f64) -> Self {
        LinearKalmanFilter {
            x: initial_x,
            p: initial_p,
            k: 0.0,
        }
    }

    fn update(mut self, measurement: f64, process_noise: f64, measurement_noise: f64) -> Self {
        // 预测
        let x_pred = self.x; // 线性模型假设状态不变
        let p_pred = self.p + process_noise;

        // 更新
        self.k = p_pred / (p_pred + measurement_noise);

        // 融合测量
        self.x = x_pred + self.k * (measurement - x_pred);

        // 更新误差协方差
        self.p = (1.0 - self.k) * p_pred;

        self
    }
}

fn main() {
    let linear_data = vec![
        99.842, 100.123, 99.987, 
        100.456, 99.654, 99.995, 
        100.321, 99.876, 100.234, 
        99.789, 98.996, 99.256, 
        101.024, 99.654, 100.789,
    ];

    let mut kf = LinearKalmanFilter::new(100.0, 1.0);

    // 定义过程噪声（模型不确定性）和测量噪声（传感器误差） 
    let (process_noise, measurement_noise) = (0.01, 0.2);

    let filtered_data: Vec<f64> = linear_data.iter().map(|&measurement| {
        kf = kf.update(measurement, process_noise, measurement_noise);
        println!("测量: {:.3}, 估计: {:.3}, 卡尔曼增益: {:.3}", measurement, kf.x, kf.k);
        kf.x
    }).collect();

    println!("原始数据: {:?}", linear_data);
    print!("[");
    for (index, i) in linear_data.iter().enumerate() {
        print!("({}, {}), ", index + 1, i);
    }
    println!("]");
    println!("滤波后的数据: {:?}", filtered_data);
    print!("[");
    for (index, i) in filtered_data.iter().enumerate() {
        print!("({}, {}), ", index + 1, i);
    }
    println!("]");
}
```

此外，还可以测试 EKF 和 UKF 等非线性滤波器，来处理更复杂的系统状态估计问题。

下面演示一个 EKF 跟踪 $sin x$ 函数的例子：

```rust
use math_matrix::Matrix;
use std::fs::File;
use std::io::Write;

#[derive(Clone)]
struct EKF {
    x: Matrix,  // 状态向量
    p: Matrix,  // 误差协方差矩阵
    q: Matrix,  // 过程噪声协方差矩阵
    r: f64,     // 测量噪声协方差
}

impl EKF {
    fn new(init_theta: f64, init_omega: f64) -> Self {
        EKF {
            x: Matrix::new(vec![init_theta, init_omega], (2, 1)).unwrap(),
            p: Matrix::identity_matrix(2),
            q: Matrix::new(vec![0.01, 0.0, 0.0, 0.1], (2, 2)).unwrap(),
            r: 0.01,
        }
    }

    fn kalman_update_scaler(&mut self, y_meas: f64) {
        // 预测测量值
        let z_pred = h(self.x.get(1, 1).unwrap());
        // 测量残差
        let y = y_meas - z_pred;

        // 计算雅可比矩阵 H
        #[allow(non_snake_case)]
        let mut H = Matrix::new(vec![0.0, 0.0], (1, 2)).unwrap();
        build_h(self.x.get(1, 1).unwrap(), &mut H);

        // 新息协方差 S = H * P * H^T + R
        #[allow(non_snake_case)]
        let S = H.clone() * self.p.clone() * H.transpose() + Matrix::new(vec![self.r], (1, 1)).unwrap();

        // 卡尔曼增益 K = P * H^T * S^-1
        #[allow(non_snake_case)]
        let K = self.p.clone() * H.transpose() * S.inverse().unwrap();

        // 更新状态估计 x = x + K * y
        let ky = K.clone() * Matrix::new(vec![y], (1, 1)).unwrap();
        self.x = self.x.clone() + ky;

        // 更新误差协方差 P = (I - K * H) * P
        self.p = (Matrix::identity_matrix(2) - K * H) * self.p.clone();
    }

    // EKF 的预测步骤
    fn predict(&mut self, dt: f64) {
        // 构建状态转移矩阵 F
        #[allow(non_snake_case)]
        let mut F = Matrix::new(vec![0.0; 4], (2, 2)).unwrap();
        build_f(dt, &mut F);

        // 预测状态 x = F * x
        let x_pred = F.clone() * self.x.clone();
        self.x = x_pred;

        // 预测误差协方差 P = F * P * F^T + Q
        self.p = F.clone() * self.p.clone() * F.transpose() + self.q.clone();
    }
}

// 构建状态转移矩阵 F
#[allow(non_snake_case)]
fn build_f(dt: f64, F: &mut Matrix) {
    let _ = F.set(1, 1, 1.0);
    let _ = F.set(1, 2, dt);
    let _ = F.set(2, 1, 0.0);
    let _ = F.set(2, 2, 1.0);
}

// 观测函数 h(x) -> sin(theta)
fn h(theta: f64) -> f64 {
    theta.sin()
}

// 构建雅可比矩阵 H
#[allow(non_snake_case)]
fn build_h(theta: f64, H: &mut Matrix) {
    let _ = H.set(1, 1, theta.cos());   // dh/dtheta = cos(theta)
    let _ = H.set(1, 2, 0.0);           // dh/domega = 0
}

fn main() {
    // 初始化 EKF，跟踪步长为 0.1 秒，跟踪函数 h(x) = sin(x)
    let mut ekf = EKF::new(0.0, 1.0);

    let dt = 0.1; // 时间步长
    let mut t = 0.0; // 初始时间
    let mut true_theta: f64 = 0.0;
    let true_omega = 1.0;

    // 生成 10000 个模拟测量数据
    let mut measurements = Vec::new();
    let mut simu_theta = true_theta;
    for _ in 0..10000 {
        // 模拟系统的真实状态
        simu_theta += true_omega * dt;

        // 模拟测量值，添加高斯噪声
        let noise = rand::random::<f64>() * 0.1 - 0.05; // [-0.05, 0.05]
        let measurement = h(simu_theta) + noise;
        measurements.push(measurement);
    }

    // 使用 EKF 进行状态估计（按生成顺序处理测量值）
    let mut filtered_measurements = Vec::new();
    for measurement in measurements.clone() {
        ekf.predict(dt);

        // 更新真实状态
        true_theta += true_omega * dt;

        ekf.kalman_update_scaler(measurement);

        // 打印当前时间、真实状态、测量值和 EKF 估计值
        println!(
            "时间: {:.2} s, 真实角度: {:.3} rad, 测量值: {:.3}, EKF 估计值: {:.3}",
            t,
            true_theta,
            measurement,
            h(ekf.x.get(1, 1).unwrap())
        );
        filtered_measurements.push(h(ekf.x.get(1, 1).unwrap()));
        t += dt;
    }

    // 写入原始测量值和滤波后的估计值到文件
    let mut file = File::create("ekf_output.txt").expect("无法创建文件");
    writeln!(file, "原始测量值:").unwrap();
    write!(file, "[").unwrap();
    for (index, measurement) in measurements.iter().enumerate() {
        write!(file, "({:.1}, {:.3}), ", (index + 1) as f64 * 0.1, measurement).unwrap();
    }
    writeln!(file, "]").unwrap();
    writeln!(file, "滤波后的估计值:").unwrap();
    write!(file, "[").unwrap();
    for (index, estimate) in filtered_measurements.iter().enumerate() {
        write!(file, "({:.1}, {:.3}), ", (index + 1) as f64 * 0.1, estimate).unwrap();
    }
    writeln!(file, "]").unwrap();
}
```

依照 EKF 的公式编写矩阵运算，构建状态转移矩阵 F 和雅可比矩阵 H，来处理非线性系统的状态估计问题。通过模拟测量数据并使用 EKF 进行滤波，可以看到 EKF 对于跟踪非线性函数（如 sin x）非常有效。

数据如下：

```text
原始测量值:
[(0.1, 0.055), (0.2, 0.162), (0.3, 0.304), (0.4, 0.354), (0.5, 0.528), (0.6, 0.552), (0.7, 0.648), ...
滤波后的估计值:
[(0.1, 0.100), (0.2, 0.199), (0.3, 0.296), (0.4, 0.389), (0.5, 0.479), (0.6, 0.565), (0.7, 0.644), ...
```

取一部分值来观察滤波效果，可以看到 EKF 的估计值更接近于真实的 sin x 函数值，而原始测量值由于噪声的影响，偏离较大。

![EKF 滤波结果示例](https://cdn.jsdelivr.net/gh/zhangnjsd/img-jsdelivr-go/kalman/k2.png)

![EKF 滤波结果示例](https://cdn.jsdelivr.net/gh/zhangnjsd/img-jsdelivr-go/kalman/k3.png)

![EKF 滤波结果示例](https://cdn.jsdelivr.net/gh/zhangnjsd/img-jsdelivr-go/kalman/k4.png)

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
