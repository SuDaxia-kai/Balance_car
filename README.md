# 摸鱼小项目(一) ---- 两轮平衡车

## 硬件选型

- 电机：MG513P30_12V

- 编码器

## 数学建模

| 变量 | 大小 | 说明              |
| ---- | ---- | ----------------- |
| $L$  | 0.06 | 车子的重心高度$m$ |

对于输入外部扰动时选择的量为角加速度$\ddot{\theta}$



TIM6定时中断：

 $\text{It}_{time} =\frac{(Pre+1)(Counter+1)}{Tclk}=5ms$

左轮：A 

右轮：B 

TIM2: set_pwm_2  A

TIM3: set_pwm_1  B

set_pwm_1  负的代表正着走

set_pwm_2  正的代表正着走



姿态结算频率：200hz

控制频率        ：100hz

对于MIT IMU的机械中值是-1°



对偶，确实，其实我不应该选择自然情况下的$0^o$点，而应该选择机械中值！理应如此。

其实有一个问题，那就是我如何转换我的加速度到我的电机上面呢？ --- 目前的解决方法是直接将加速度的值变换成 pwm 值

还有一个就是说 MPU6050 返回的 Pitch 角度是会有误差的， 目前是 $10^{-1}$精度的误差，太恐怖了，假设 kp = 200, 相当于误差被拉了 20，算出来会有$ m/s^2$,所以目前解决的办法是通过取整，直接对 pitch 角度取整，但其实这丧失了响应的快速性，离散的太大了，得拉大kd补回来



## 对于直立环$K_p$的调节

对平衡车进行稳态误差分析如下：
$$
\begin{align*}
e_s&=\lim_{t \to \infty}[r(t)-y(t)]=\lim_{t \to \infty}[r(t)-\phi(t) r(t)]=\lim_{s \to 0}s[R(s)-\Phi(s) R(s)] \\
   &=\lim_{s \to 0}\frac{sR(s)}{1+Q(s)}
\end{align*}
$$
KP = 280

<img src="C:\Users\SuDaxia\AppData\Roaming\Typora\typora-user-images\image-20230426005315789.png" alt="image-20230426005315789" style="zoom:50%;" />

KP =290

<img src="C:\Users\SuDaxia\AppData\Local\Temp\WeChat Files\dbd44bc182e029f9ff9e223a3ea4f79.png" alt="dbd44bc182e029f9ff9e223a3ea4f79" style="zoom:50%;" />

KP = 320

<img src="C:\Users\SuDaxia\AppData\Roaming\Typora\typora-user-images\image-20230426005542158.png" alt="image-20230426005542158" style="zoom:50%;" />

0.75

## 速度环

对编码器的值进行一阶低通滤波，目的是为了不让速度产生骤变，让速度更加平滑。

## 卡尔曼滤波

电机参数如下
