# 摄像头标定( camera calibration )
摄像头摄像头使用前必须进行。摄像头矩阵分为外参矩阵和内参矩阵。其中内参矩阵表示将小孔为原点的相机坐标投影到像素坐标的矩阵.
外参矩阵表示，将世界坐标系投影到相机坐标系的矩阵。

内参矩阵只与摄像头硬件有关，只要摄像机生产组装完毕，它的一切内参矩阵参数都是确定的，我们只用标定一次即可。它的一般形式如下:
$$
    T = \begin{bmatrix}
        fx & 0  & cx\\
        0  & fy & cy\\
        0  & 0  &  1\\ 
        \end{bmatrix}
$$

外参数矩阵与世界坐标系统的选取有关，外参矩阵由摄像头坐标系在世界坐标系中的位置$t$和姿态$R$组成。

## 摄像头的成像原理
摄像头的成像原理很简单，就是小孔成像来的。根据小孔成像模型，假设焦距(并不是相机的那个焦距)为f，那么在小孔为原点的坐标系下一点P(x,y,z)投影到成像平面一点p'(x', y')，他们之间的关系为:
$$
   \begin{align} 
        \frac{z}{f} = \frac{x}{x'} \\ 
        \frac{z}{f} = \frac{y}{y'}
   \end{align}
$$

根据上式，进一步，可以得到世界坐标下一点和投影到成像平面坐标的关系:
$$
   \begin{align} 
        x' = \frac{fx}{z} \\ 
        y' = \frac{fy}{z} \\ 
   B
   \end{align}
$$

这样投影下的成像平面坐标的原点在图像的中间，并不满足我们的要求，需要进行平移操作，将原点移动到左下角。 此外， 这个成像平面的坐标单位为m，而像素坐标的单位为像素，于是中间还存在一个单位转换，通过硬件CCD转换单位。于是，修正为:
$$
   \begin{align} 
        x' = \frac{f_x x}{z} + c_x \\ 
        y' = \frac{f_y y}{z} + c_y\\ 
   \end{align}
$$

其实，小孔成像存在很多局限性，比如光线等问题，需要增加一个透镜辅助成像，于是之前说的焦距还存在偏差，但是我们只进行标定，不用管。根据上式，我们可以将两边的坐标改写成为齐次坐标，写成矩阵形式:
$$
    \begin{align}
        \begin{bmatrix} x' \\ y'\\ z \end{bmatrix} =
        \begin{bmatrix}  
            fx & 0  & cx & 0\\
            0  & fy & cy & 0\\
            0  & 0  &  1 & 0\\ 
        \end{bmatrix}
    \end{align}
$$
通常我们将上面的转换矩阵写成$ T = M[1 0] $。

## 摄像头非线性成像模型
由于实际成像存在各种影响，实际使用中，摄像头并不一定满足小孔成像的原理，可能存在几个像素的畸变，这个时候就需要对这种非线性模型进行建模，如下公式：

$$
\begin{align}
    \left\{
    \begin{aligned}
    x = y_d + \delta_x \\ 
    y = y_d + \delta_y
    \end{aligned}
    \right.
\end{align}
$$

其中，$(x,y)$ 是针孔模型成像条件下的图像点的坐标，$ \delta_x和\delta_y$分别为$X$和$Y$方向上的畸变值，与图像点的位置有关。

在标定中应用广泛的是Tsai和Weng's他们两人的方法。其中Weng's对Tsai的畸变模型进行改进拓展，分别考虑了轴对称畸变、偏心畸变和薄棱镜畸变。它么的模型为:

轴对称畸变:
$$
    \begin{align}
    \left\{
    \begin{aligned}
    \delta_{xr} = x_u(k_1\rho ^2 + k_2 \rho ^4 + \cdots + ) \\ 
    \delta_{yr} = y_u(k_1\rho ^2 + k_2 \rho ^4 + \cdots + )
    \end{aligned}
    \right.
    \end{align}
$$

其中,$ \rho = \sqrt{x_d^2 + y_d ^2}$ 是图像点到原点的距离，$k_1 \text{和} k_2$表示轴对称误差的系数。

偏心畸变:
$$
    \begin{align}
    \left\{
    \begin{aligned}
    \delta_{xr} = k_1 x_d(x_d^2+y_d^2) + O[(x_d,y_d)^5] \\ 
    \delta_{yr} = k_1 x_d(x_d^2+y_d^2) + O[(x_d,y_d)^5] \\ 
    \end{aligned}
    \right.
    \end{align}
$$

## 摄像头标定
摄像头标定就是标定外参和内参的过程，我们要使用摄像头就要标定得足够准确才能保证后续工作的稳定。



| 标定方法 | 优点 | 缺点 |
| --- | --- | --- |
| Tai | j | s |
