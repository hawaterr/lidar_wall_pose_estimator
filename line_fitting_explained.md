# Fitting a Line to 2D Points (Total Least Squares)

```cpp
float phi = 0.5 * atan2(2*sxy, sxx - syy); // line direction
float a   = phi + M_PI/2;                  // normal angle
float d   = cos(a)*mx + sin(a)*my;         // distance to line
```

## 1. Covariance Matrix

For $n$ points $(x_i, y_i)$ with centroid $(m_x, m_y)$:

$$
\Sigma =
\begin{bmatrix}
s_{xx} & s_{xy} \\
s_{xy} & s_{yy}
\end{bmatrix},
\qquad
\begin{aligned}
s_{xx} &= \textstyle\sum (x_i - m_x)^2 \\
s_{yy} &= \textstyle\sum (y_i - m_y)^2 \\
s_{xy} &= \textstyle\sum (x_i - m_x)(y_i - m_y)
\end{aligned}
$$

$\Sigma$ describes how the points are spread around the centroid:

- Largest eigenvalue's eigenvector → **line direction** (the direction of maximum spread).
- Smallest eigenvalue's eigenvector → **line normal** (the direction of minimum spread, i.e. the fitting error).

[[1](https://www.youtube.com/watch?v=MNbJTYnYN0E)]

## 2. Line Angle $\varphi$

The variance of the points projected onto a direction $u = (\cos\theta, \sin\theta)^T$ is:

$$
V(\theta) = u^T \Sigma u = s_{xx}\cos^2\theta + 2 s_{xy}\cos\theta\sin\theta + s_{yy}\sin^2\theta
$$

The line direction is where $V$ is largest. Setting $dV/d\theta = 0$:

$$
\tan(2\theta) = \frac{2\,s_{xy}}{s_{xx} - s_{yy}}
\quad\Rightarrow\quad
\varphi = \tfrac{1}{2}\,\text{atan2}\!\left(2\,s_{xy},\; s_{xx} - s_{yy}\right)
$$

`atan2` uses the signs of both arguments to return the **maximum** (line direction) rather than the minimum (normal).

## 3. Distance $d$

With the normal angle $\alpha = \varphi + \frac{\pi}{2}$, the line in Hesse normal form is:

$$
x\cos\alpha + y\sin\alpha = d
$$

- The left side, $\mathbf{n}\cdot\mathbf{p}$ with $\mathbf{n} = (\cos\alpha, \sin\alpha)$, is how far point $\mathbf{p}$ lies **along the normal**.
- Every point on the line has the same value for this, and that value is $d$.
- $d$ is the signed perpendicular distance from the origin to the line.

The best-fit line passes through the centroid, so plugging in $(m_x, m_y)$ gives $d$:

$$
d = m_x\cos\alpha + m_y\sin\alpha
$$

## 4. Notes

- https://www.youtube.com/watch?v=MNbJTYnYN0E
