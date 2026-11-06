# 四旋翼 FPV 动力系统整机拉力测试与推力-油门建模工程规范
## （基于 RPM 解耦、标准大气环境与基准电压归一化）

---

## 1. 理论基础与物理数学模型

### 1.1 基准参考状态定义
为了消除测试现场天气与放电压降的偶发扰动，定义全局统一的基准标定工况：
* **标准气象环境（STP）**：
  * 基准温度：$T_0 = 273.15\text{ K}$（$0^\circ\text{C}$）
  * 基准气压：$P_0 = 101325\text{ Pa}$（$1\text{ atm}$）
* **标准电气基准**：
  * 标称参考母线电压：$V_{\text{ref}} = 22.20\text{ V}$（6S 锂电池组标称电压）

---

### 1.2 环境气象推力修正（消去气体常数）
螺旋桨在定常流动中的空气动力学拉力（推力）$F$ 服从动量与叶素理论：

$$F = C_T \rho \left(\frac{\text{RPM}}{60}\right)^2 D^4$$

定距螺旋桨在转速不变时，推力与空气密度成正比（$F \propto \rho$）。根据理想气体状态方程：

$$P = \rho R_{\text{spec}} T \implies \rho = \frac{P}{R_{\text{spec}} T}$$

在实测环境（绝对温度 $T_{\text{meas}} = t_{\text{meas}} + 273.15\text{ K}$，压强 $P_{\text{meas}}$）与标准环境（$T_0, P_0$）之间进行推力归一化时，空气比气体常数 $R_{\text{spec}}$ 在分子分母中**严格对消**：

$$\frac{\rho_0}{\rho_{\text{test}}} = \frac{\frac{P_0}{R_{\text{spec}} T_0}}{\frac{P_{\text{meas}}}{R_{\text{spec}} T_{\text{meas}}}} = \frac{P_0}{P_{\text{meas}}} \cdot \frac{T_{\text{meas}}}{T_0}$$

因此，无需计算实际空气密度数值，实测拉力 $F_{\text{meas}}$ 映射到标准环境下的推力 $F_{\text{std\_env}}$ 可直接表示为：

$$F_{\text{std\_env}} = F_{\text{meas}} \cdot \left( \frac{P_0}{P_{\text{meas}}} \cdot \frac{T_{\text{meas}}}{T_0} \right) = F_{\text{meas}} \cdot \left( \frac{101325}{P_{\text{meas}}} \cdot \frac{t_{\text{meas}} + 273.15}{273.15} \right)$$

---

### 1.3 四电机转速均方根（RMS RPM）解耦原理
由于四旋翼整机测试中各电机制造公差、气流动平衡或电调校准存在微小差异，四旋翼总拉力满足：

$$F_{\text{total}} = \sum_{k=1}^4 F_k = C_T \rho D^4 \sum_{k=1}^4 \left(\frac{\text{RPM}_k}{60}\right)^2$$

定义整机**有效等效转速**为均方根转速 $\text{RPM}_{\text{rms}}$：

$$\text{RPM}_{\text{rms}} \triangleq \sqrt{\frac{1}{4} \sum_{k=1}^4 \text{RPM}_k^2}$$

则总推力可严格化简为单一等效转速表达式：

$$F_{\text{total}} = 4 \cdot C_T \rho D^4 \left(\frac{\text{RPM}_{\text{rms}}}{60}\right)^2 = K_{\text{aero}} \cdot \rho \cdot \text{RPM}_{\text{rms}}^2$$

**核心解耦逻辑**：转速 $\text{RPM}_{\text{rms}}$ 与推力 $F_{\text{total}}$ 的关系纯粹取决于气动外形与空气密度，与电池电压、内阻无电气关联；而油门 $u$、母线电压 $V$ 与电流 $I$ 则通过电机电气动力方程唯一确定 $\text{RPM}_{\text{rms}}$。

---

### 1.4 电气动力学方程与母线压降折算
对于采用主动续流（Damped Light）的无刷驱动系统，整机动力系统的等效反电动势平衡方程为：

$$E_b = K_e \cdot \text{RPM}_{\text{rms}} = u \cdot V_{\text{meas}} - I_{\text{meas}} \cdot R_{\text{eq}}$$

式中：
* $u \in [0, 1]$ 为整机归一化油门开度（PWM 占空比）；
* $V_{\text{meas}}$ 为当前母线实时电压；
* $I_{\text{meas}}$ 为全机总回路电流；
* $R_{\text{eq}}$ 为供电母线、分电板、电调内阻及四路电机并联绕组的**全系统等效集中电阻**；
* $K_e$ 为整机电机的反电动势常数（$K_e \approx \frac{1}{K_v}$）。

当母线电压恒定在基准值 $V_{\text{ref}} = 22.20\text{ V}$ 时，对于指定的油门 $u_i$，其对应的标准工作转速 $\text{RPM}_{\text{std}, i}$ 满足：

$$\text{RPM}_{\text{std}, i} = \text{RPM}_{\text{rms}, i} \cdot \frac{u_i V_{\text{ref}} - I_{\text{std}, i} R_{\text{eq}}}{u_i V_i - I_i R_{\text{eq}}}$$

在空气动力学负载下，稳态转矩与电流成正比，且与转速平方成正比，即 $I \propto \text{RPM}^2$。由此可解出标准状态下的推力虚拟测量值点集 $(u_i, F_{\text{std}, i})$。

---

### 1.5 标准推力-油门函数模型（无截距三次多项式）
在基准状态（$V_{\text{ref}} = 22.2\text{ V}, T_0 = 273.15\text{ K}, P_0 = 101325\text{ Pa}$）下，总推力与油门的关系拟合为：

$$F_{\text{std}}(u) = a_3 u^3 + a_2 u^2 + a_1 u$$

**模型约束解析**：
1. **强制零截距（Zero-Intercept）**：常数项 $a_0 \equiv 0$。保证当油门 $u = 0$ 时，理论推力严格为 0，防止飞控怠速与切断输出时产生虚假推力。
2. **为何引入三次项**：理想动量理论为二次关系（$u^2$），但随着油门增大，大电流在内阻 $R_{\text{eq}}$ 上引起的电压跌落以及桨叶叶尖失速效应会带来“高油门软化”，三次项 $a_3 u^3$ 能精确拟合该非线性衰减特性。

---

## 2. 试验台数据采集方案 (Test SOP)

### 2.1 硬件台架与测试连接
1. **机械固定**：整机通过专用夹具固定在台称上，确保拉力方向垂直于传感器受力轴线，消除弯矩分量。
2. **测点布置**：
   * **气动拉力**：使用台称记录相对于起桨前减小的重量，计算出整机总拉力 $F$（采样率 $\ge 10\text{ Hz}$）。
   * **四电机转速**：开启 Betaflight 的 **双向 DShot（Bidirectional DShot）**，通过飞控读取四路真实的物理 $\text{RPM}_1 \sim \text{RPM}_4$。
   * **电参数**：读取飞控的总电流 $I$，电池电压 $V$。
   * **气象参数**：通过飞控读取环境气压与温度，记录环境温度 $t_{\text{meas}}$（$^\circ\text{C}$）和绝对气压 $P_{\text{meas}}$（$\text{Pa}$）。

```
        +-------------------------------------------------------+
        |                  试验台数据采集架构                    |
        +-------------------------------------------------------+
                                    |
         [环境传感器]               |          [动力电气回路]
         - 温度 t_meas (℃)          |          - 母线电压 V_meas (V)
         - 气压 P_meas (Pa)         |          - 总回路电流 I_meas (A)
                |                   |                 |
                v                   |                 v
     +--------------------+         |        +--------------------+
     | 环境推力缩放系数   |         |        | 等效反电动势解耦   |
     | (P0/P) * (T/T0)    |         |        |   E_b = uV - I*Req |
     +--------------------+         |        +--------------------+
                |                   |                 |
                +-----------> [ 数据归一化引擎 ] <-----+
                                    ^
                                    |
         [气动力学与执行器]          |
         - 双向 DShot RPM_1~4       |
         - 总拉力传感器 F_meas (N)   |
                                    |
```

### 2.2 阶梯测试动作设计
为了兼顾动平衡稳定与热衰退控制，采用阶梯递增测试：
1. **油门步进**：从 $u = 0.05$（怠速启动）开始，以 $\Delta u = 0.05$ 为步长递增至 $u = 1.00$ （可调），共 20 个测试阶梯。
2. **阶跃驻留时间**：
   * 每个油门台阶总持续时间：$2.5\text{ s}$。
   * **稳定等待段（前 $0.5\text{ s}$）**：等待气流涡环稳定和力传感器机械谐振衰减，数据予以丢弃。
   * **稳态采样段（后 $2.0\text{ s}$）**：在该区间内进行高频同步均值滤波，输出单组稳态数据点 $(u_i, V_i, I_i, \text{RPM}_{1 \sim 4, i}, F_i)$。
3. **降温与去极化间隙**：
   * 在 $u \ge 0.70$ 的大功率区间，每个台阶测试完毕后自动回落至怠速运行 $5.0\text{ s}$，吹风冷却电机绕组并减缓电池极化，防止高温引起电机磁钢退磁与相阻漂移。

---

## 3. 数据归一化与多项式拟合步骤

### 3.1 离散点归一化处理流水线
对于采集到的第 $i$ 组实测数据 $(u_i, V_i, I_i, \text{RPM}_{1,i}, \text{RPM}_{2,i}, \text{RPM}_{3,i}, \text{RPM}_{4,i}, F_i, P_i, t_{\text{meas}, i})$：

1. **环境推力归一化（直接消去气体常数）**：
   $$F_{0, i} = F_i \cdot \left( \frac{101325}{P_i} \cdot \frac{t_{\text{meas}, i} + 273.15}{273.15} \right)$$

2. **转速均方根合成**：
   $$\text{RPM}_{\text{rms}, i} = \sqrt{\frac{1}{4} \sum_{k=1}^4 \text{RPM}_{k, i}^2}$$

3. **系统内阻 $R_{\text{eq}}$ 与电机反电动势常数辨识**：
   利用不同油门下的多组 $(u_i V_i, I_i, \text{RPM}_{\text{rms}, i})$ 构建立体线性回归方程：
   $$u_i V_i = \left(\frac{1}{K_v^*}\right) \text{RPM}_{\text{rms}, i} + R_{\text{eq}} I_i$$
   通过多元线性最小二乘法，解得系统的综合等效内阻 $R_{\text{eq}}$ 及综合转速常数 $K_v^*$。

4. **折算到标准电压 $22.2\text{ V}$ 下的标准推力**：
   基于气动外特性 $F \propto \text{RPM}^2$ 及电枢方程，计算当前油门 $u_i$ 在 $22.2\text{ V}$ 恒定供电时的等效推力：
   $$F_{\text{std}, i} = F_{0, i} \cdot \left( \frac{u_i \cdot 22.20 - I_{\text{std}, i} R_{\text{eq}}}{u_i \cdot V_i - I_i R_{\text{eq}}} \right)^2$$
   *注：其中 $I_{\text{std}, i} \approx I_i \cdot \left(\frac{22.20}{V_i}\right)^2$，若电流压降占比极小，可采用一阶紧缩公式：$F_{\text{std}, i} \approx F_{0, i} \cdot \left(\frac{22.20}{V_i}\right)^2$。*

---

### 3.2 无截距三次多项式加权最小二乘求解
获得归一化数据集 $\{ (u_i, F_{\text{std}, i}) \}_{i=1}^N$（$N \ge 20$），拟合目标函数：

$$F_{\text{std}}(u) = a_3 u^3 + a_2 u^2 + a_1 u$$

构建设计矩阵 $\mathbf{X} \in \mathbb{R}^{N \times 3}$ 和目标向量 $\mathbf{Y} \in \mathbb{R}^{N \times 1}$：

$$\mathbf{X} = \begin{bmatrix}
u_1^3 & u_1^2 & u_1 \\
u_2^3 & u_2^2 & u_2 \\
\vdots & \vdots & \vdots \\
u_N^3 & u_N^2 & u_N
\end{bmatrix}, \quad
\mathbf{Y} = \begin{bmatrix}
F_{\text{std}, 1} \\
F_{\text{std}, 2} \\
\vdots \\
F_{\text{std}, N}
\end{bmatrix}, \quad
\mathbf{\Theta} = \begin{bmatrix} a_3 \\ a_2 \\ a_1 \end{bmatrix}$$

通过正则正规方程（Normal Equations）求解参数解向量：

$$\mathbf{\Theta} = (\mathbf{X}^T \mathbf{X})^{-1} \mathbf{X}^T \mathbf{Y}$$

#### 物理合法性与单调性校验
求得系数向量 $[a_3, a_2, a_1]^T$ 后，必须在定义域 $u \in [0, 1]$ 内执行微积分单调性检验：
1. **导数严格为正**：
   $$\frac{d F_{\text{std}}}{d u} = 3 a_3 u^2 + 2 a_2 u + a_1 > 0 \quad (\forall u \in [0, 1])$$
2. **原点正斜率**：$a_1 > 0$（保证极低油门具有正向推力增益响应）。

---

## 4. 在线实时反解与控制算法 (Real-Time Inference)

在飞控运行阶段，控制环路输入包括：当前环境温度 $t_{\text{cur}}$（$^\circ\text{C}$）、当前气压 $P_{\text{cur}}$（$\text{Pa}$）、实时母线电压 $V_{\text{meas}}$、实时总电流 $I_{\text{meas}}$，以及姿态/高度控制器期望输出的总推力 $F_{\text{req}}$。

算法执行流程如下：

```
           +---------------------------------------------+
           |       输入: F_req, t_cur, P_cur, V_meas     |
           +---------------------------------------------+
                                  |
                                  v
     +---------------------------------------------------------+
     | 步骤 1: 环境直接比值折算 (无需 R_spec)                  |
     | F_std_target = F_req * (101325/P_cur) * (T_cur/273.15)  |
     +---------------------------------------------------------+
                                  |
                                  v
     +---------------------------------------------------------+
     | 步骤 2: 牛顿-拉夫逊数值反解                              |
     | 解方程: a3*u^3 + a2*u^2 + a1*u - F_std_target = 0        |
     | 迭代 3~4 次快速收敛至 u_std                              |
     +---------------------------------------------------------+
                                  |
                                  v
     +---------------------------------------------------------+
     | 步骤 3: 动态电气压降与内阻补偿                           |
     | u_cmd = (u_std * 22.20 + (I_meas - I_std)*Req) / V_meas |
     +---------------------------------------------------------+
                                  |
                                  v
     +---------------------------------------------------------+
     | 步骤 4: 饱和限幅与安全输出                               |
     | u_out = clamp(u_cmd, u_idle, 1.0)                       |
     +---------------------------------------------------------+
```

### 步骤 1：将期望推力折算为标准环境推力
利用环境压强与绝对温度比值，直接将当前期望推力折算至标准大气环境下的目标推力 $F_{\text{std\_target}}$：

$$F_{\text{std\_target}} = F_{\text{req}} \cdot \left( \frac{P_0}{P_{\text{cur}}} \cdot \frac{t_{\text{cur}} + 273.15}{T_0} \right) = F_{\text{req}} \cdot \left( \frac{101325}{P_{\text{cur}}} \cdot \frac{t_{\text{cur}} + 273.15}{273.15} \right)$$

### 步骤 2：基准油门反解（高效牛顿-拉夫逊迭代）
在标准环境与标准 $22.2\text{ V}$ 下，求解未知数 $u_{\text{std}}$：

$$f(u) = a_3 u^3 + a_2 u^2 + a_1 u - F_{\text{std\_target}} = 0$$

其一阶导数（推力雅可比）：

$$f'(u) = 3 a_3 u^2 + 2 a_2 u + a_1$$

由于 $F(u)$ 在 $[0, 1]$ 上单调递增，采用牛顿迭代法可在 $3 \sim 4$ 步内达到 $10^{-5}$ 精度。
* **初值选取**：$u^{(0)} = \sqrt{\frac{F_{\text{std\_target}}}{a_3 + a_2 + a_1}}$
* **迭代公式**：
  $$u^{(k+1)} = u^{(k)} - \frac{a_3 (u^{(k)})^3 + a_2 (u^{(k)})^2 + a_1 u^{(k)} - F_{\text{std\_target}}}{3 a_3 (u^{(k)})^2 + 2 a_2 u^{(k)} + a_1}$$

### 步骤 3：母线电压与电流负载动态补偿
求得标准油门 $u_{\text{std}}$ 后，根据反电动势等效原理，实时目标油门 $u_{\text{cmd}}$ 应补偿当前母线跌落及相阻压降：

$$u_{\text{cmd}} = \frac{u_{\text{std}} \cdot 22.20 + (I_{\text{meas}} - I_{\text{std}}) R_{\text{eq}}}{V_{\text{meas}}}$$

若在飞控高速循环（如 $1\text{ kHz}$）中不引入电流反馈，一阶工程电压补偿公式为：

$$u_{\text{cmd}} = u_{\text{std}} \cdot \left( \frac{22.20}{V_{\text{meas\_lpf}}} \right)$$

*注意：$V_{\text{meas\_lpf}}$ 必须经过截止频率在 $2\text{ Hz} \sim 5\text{ Hz}$ 的一阶低通滤波，防止大油门突加时电压瞬降形成正反馈发散。*

### 步骤 4：死区处理与饱和限幅
$$u_{\text{final}} = \text{clip}\left( u_{\text{cmd}}, \; u_{\text{idle}}, \; 1.0 \right)$$

---

## 5. C 语言核心驱动代码参考

```c
#include <math.h>

/**
 * @brief 动力系统标定模型与环境参数结构体
 */
typedef struct {
    // 拟合模型参数: F(u) = a3*u^3 + a2*u^2 + a1*u (标准状态: 22.2V, 0℃, 101325Pa)
    float a3;
    float a2;
    float a1;
    
    float v_ref;        // 基准电压，固定为 22.20f
    float u_idle;       // 怠速死区下限，例如 0.05f (5%)
    float v_lpf_alpha;  // 电压低通滤波系数
    float v_filtered;   // 滤波后的母线电压
} PropulsionModel_t;

/**
 * @brief 初始化动力模型
 */
void Propulsion_Init(PropulsionModel_t *model, float a3, float a2, float a1, float u_idle) {
    model->a3 = a3;
    model->a2 = a2;
    model->a1 = a1;
    model->v_ref = 22.20f;
    model->u_idle = u_idle;
    model->v_lpf_alpha = 0.02f; // 对应约 3Hz 截止频率 (在 1kHz 循环中)
    model->v_filtered = 22.20f;
}

/**
 * @brief 在线计算所需油门数值
 * 
 * @param model        标定模型句柄
 * @param f_req        期望总推力/拉力 (N)
 * @param t_celsius    当前环境温度 (摄氏度)
 * @param p_pa         当前大气压强 (Pa)
 * @param v_raw        当前实时母线采样电压 (V)
 * @return float       最终输出给电调的控制油门 u ∈ [u_idle, 1.0]
 */
float Propulsion_CalculateThrottle(PropulsionModel_t *model, 
                                   float f_req, 
                                   float t_celsius, 
                                   float p_pa, 
                                   float v_raw) 
{
    // 0. 负推力或零推力安全保护
    if (f_req <= 0.001f) {
        return model->u_idle;
    }

    // 1. 低通滤波母线电压，消除高频电流动态压降引起的正反馈震荡
    model->v_filtered += model->v_lpf_alpha * (v_raw - model->v_filtered);
    if (model->v_filtered < 12.0f) {
        model->v_filtered = 12.0f; // 硬件欠压保护下限
    }

    // 2. 环境气象推力折算 (依据理想气体状态方程，R_spec 分子分母直接对消)
    // F_std_target = F_req * (P0 / P_cur) * (T_cur / T0)
    const float P0 = 101325.0f;
    const float T0 = 273.15f;
    float t_kelvin = t_celsius + 273.15f;
    float f_std_target = f_req * (P0 / p_pa) * (t_kelvin / T0);

    // 3. 牛顿-拉夫逊法反解三次多项式: a3*u^3 + a2*u^2 + a1*u - f_std_target = 0
    // 初值估算 (假设以二次项主导估算初始点)
    float u_est = sqrtf(f_std_target / (model->a3 + model->a2 + model->a1 + 1e-4f));
    if (u_est > 1.0f) u_est = 1.0f;
    if (u_est < 0.01f) u_est = 0.01f;

    // 3次牛顿迭代足以满足工业级精度要求 (<0.05% 相对误差)
    for (int i = 0; i < 3; i++) {
        float u2 = u_est * u_est;
        float u3 = u2 * u_est;
        
        float f_val = model->a3 * u3 + model->a2 * u2 + model->a1 * u_est - f_std_target;
        float f_prime = 3.0f * model->a3 * u2 + 2.0f * model->a2 * u_est + model->a1;
        
        if (fabsf(f_prime) < 1e-6f) break; // 避免奇异除零
        
        float delta = f_val / f_prime;
        u_est -= delta;
        
        // 投影限幅在 [0.0, 1.2] 之间以保证数值稳定
        if (u_est < 0.0f) u_est = 0.0f;
        if (u_est > 1.2f) u_est = 1.2f;
        
        if (fabsf(delta) < 1e-4f) break; // 提前收敛
    }

    float u_std = u_est;

    // 4. 电压补偿换算至当前工作电压
    float u_cmd = u_std * (model->v_ref / model->v_filtered);

    // 5. 饱和限幅与怠速保护
    if (u_cmd > 1.0f) {
        u_cmd = 1.0f;
    } else if (u_cmd < model->u_idle) {
        u_cmd = model->u_idle;
    }

    return u_cmd;
}
```