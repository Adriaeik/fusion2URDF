# Transformation Matrices - Assem1

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | Base0 |
| $L_{2}$ | Arm01 |
| $L_{3}$ | Arm02 |
| $L_{4}$ | for_arm_group |
| $L_{5}$ | Part02 |
| $L_{6}$ | led_base |
| $L_{7}$ | servo_mount |
| $L_{8}$ | right_pivot |
| $L_{9}$ | left_pivot |
| $L_{10}$ | acc_right_gripper |
| $L_{11}$ | acc_left_gripper |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Revolute_1 | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | Revolute_2 | continuous (rad) | $L_{1}$ | $L_{2}$ |
| $q_{3}$ | Revolute3 | continuous (rad) | $L_{2}$ | $L_{3}$ |
| $q_{4}$ | Revolute4 | continuous (rad) | $L_{3}$ | $L_{4}$ |
| $q_{5}$ | Revolute6 | revolute (rad) | $L_{4}$ | $L_{5}$ |
| $q_{6}$ | servo_joint | revolute (rad) | $L_{6}$ | $L_{7}$ |
| $q_{7}$ | left_pivot_joint2 | continuous (rad) | $L_{7}$ | $L_{8}$ |
| $q_{8}$ | right_pivot_joint2 | continuous (rad) | $L_{7}$ | $L_{9}$ |
| $q_{9}$ | left_pivot_joint1 | continuous (rad) | $L_{8}$ | $L_{10}$ |
| $q_{10}$ | right_pivot_joint1 | continuous (rad) | $L_{9}$ | $L_{11}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [continuous] Revolute_1 (q1)
      L1: Base0
        +-- [continuous] Revolute_2 (q2)
            L2: Arm01
              +-- [continuous] Revolute3 (q3)
                  L3: Arm02
                    +-- [continuous] Revolute4 (q4)
                        L4: for_arm_group
                          +-- [revolute] Revolute6 (q5)
                              L5: Part02
                                +-- [fixed] griper_joint
                                    L6: led_base
                                      +-- [revolute] servo_joint (q6)
                                          L7: servo_mount
                                            |-- [continuous] left_pivot_joint2 (q7)
                                            |   L8: right_pivot
                                            |     +-- [continuous] left_pivot_joint1 (q9)
                                            |         L10: acc_right_gripper
                                            +-- [continuous] right_pivot_joint2 (q8)
                                                L9: left_pivot
                                                  +-- [continuous] right_pivot_joint1 (q10)
                                                      L11: acc_left_gripper
```

## Transforms

## Revolute_1

$L_{0}$ **base_link** -> $L_{1}$ **Base0** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0, 0.17, 0) m
- **origin rpy**: (0, -1.134464, 0) rad
- **axis**: (0, 1, 0)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.422618 & 0 & -0.906308 & 0 \\
0 & 1 & 0 & 0.17 \\
0.906308 & 0 & 0.422618 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & 0 & s_{1} & 0 \\
0 & 1 & 0 & 0 \\
-s_{1} & 0 & c_{1} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_2

$L_{1}$ **Base0** -> $L_{2}$ **Arm01** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0.2, 0.49, 0.03) m
- **origin rpy**: (1.047198, -1.570796, 0) rad
- **axis**: (1, 0, 0)

### Local Transform

$T^{1}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.866025 & -0.5 & 0.2 \\
0 & 0.5 & -0.866025 & 0.49 \\
1 & 0 & 0 & 0.03 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{2} & -s_{2} & 0 \\
0 & s_{2} & c_{2} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute3

$L_{2}$ **Arm01** -> $L_{3}$ **Arm02** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0.1, 1, 0) m
- **origin rpy**: (-1.570796, 0, -1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{2}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & 1 & 0.1 \\
-1 & 0 & 0 & 1 \\
0 & -1 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0 \\
s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute4

$L_{3}$ **Arm02** -> $L_{4}$ **for_arm_group** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (0.189, 0.1, 0.12) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (-1, 0, 0)

### Local Transform

$$
T^{3}_{4}(q_{4}) = \begin{bmatrix}
1 & 0 & 0 & 0.189 \\
0 & c_{4} & s_{4} & 0.1 \\
0 & -s_{4} & c_{4} & 0.12 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute6

$L_{4}$ **for_arm_group** -> $L_{5}$ **Part02** (revolute)
  Variable: $q_{5}$

- **origin xyz**: (0.899, 0, 0) m
- **origin rpy**: (-3.141593, 0, 3.141593) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.745329, 1.745329] rad ([-100deg, 100deg])

### Local Transform

$T^{4}_{5}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
-1 & 0 & 0 & 0.899 \\
0 & 1 & 0 & 0 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
c_{5} & -s_{5} & 0 & 0 \\
s_{5} & c_{5} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## griper_joint

$L_{5}$ **Part02** -> $L_{6}$ **led_base** (fixed)

- **origin xyz**: (-0.152521, 0.0075, 0.0243) m
- **origin rpy**: (3.141593, 0, -1.570796) rad

### Local Transform

$$
T^{5}_{6} = \begin{bmatrix}
0 & -1 & 0 & -0.152521 \\
-1 & 0 & 0 & 0.0075 \\
0 & 0 & -1 & 0.0243 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## servo_joint

$L_{6}$ **led_base** -> $L_{7}$ **servo_mount** (revolute)
  Variable: $q_{6}$

- **origin xyz**: (0.0075, 0.0101, 0.0322) m
- **origin rpy**: (1.570796, 0, 0.07037) rad
- **axis**: (0, 1, 0)
- **limits**: [-1.012291, 0] rad ([-58deg, 0deg])

### Local Transform

$T^{6}_{7}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.997525 & 0 & 0.070312 & 0.0075 \\
0.070312 & 0 & -0.997525 & 0.0101 \\
0 & 1 & 0 & 0.0322 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
c_{6} & 0 & s_{6} & 0 \\
0 & 1 & 0 & 0 \\
-s_{6} & 0 & c_{6} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## left_pivot_joint2

$L_{7}$ **servo_mount** -> $L_{8}$ **right_pivot** (continuous)
  Variable: $q_{7}$

- **origin xyz**: (0.010607, 0.0133, 0.010607) m
- **origin rpy**: (1.570796, 0.263624, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{7}_{8}(q_{7}) = T_{fixed} \cdot R_{axis}(q_{7})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.965452 & 0.260581 & 0 & 0.010607 \\
0 & 0 & -1 & 0.0133 \\
-0.260581 & 0.965452 & 0 & 0.010607 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{7}) = \begin{bmatrix}
c_{7} & -s_{7} & 0 & 0 \\
s_{7} & c_{7} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## right_pivot_joint2

$L_{7}$ **servo_mount** -> $L_{9}$ **left_pivot** (continuous)
  Variable: $q_{8}$

- **origin xyz**: (-0.010607, 0.0093, -0.010607) m
- **origin rpy**: (-1.570796, -0.263625, -3.141593) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{7}_{9}(q_{8}) = T_{fixed} \cdot R_{axis}(q_{8})$ where:

$$
T_{fixed} = \begin{bmatrix}
-0.965452 & -0.260582 & 0 & -0.010607 \\
0 & 0 & -1 & 0.0093 \\
0.260582 & -0.965452 & 0 & -0.010607 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{8}) = \begin{bmatrix}
c_{8} & -s_{8} & 0 & 0 \\
s_{8} & c_{8} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## left_pivot_joint1

$L_{8}$ **right_pivot** -> $L_{10}$ **acc_right_gripper** (continuous)
  Variable: $q_{9}$

- **origin xyz**: (0.03, 0, 0) m
- **origin rpy**: (3.141593, 0, 0.333994) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{8}_{10}(q_{9}) = T_{fixed} \cdot R_{axis}(q_{9})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.94474 & 0.327819 & 0 & 0.03 \\
0.327819 & -0.94474 & 0 & 0 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{9}) = \begin{bmatrix}
c_{9} & s_{9} & 0 & 0 \\
-s_{9} & c_{9} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## right_pivot_joint1

$L_{9}$ **left_pivot** -> $L_{11}$ **acc_left_gripper** (continuous)
  Variable: $q_{10}$

- **origin xyz**: (0.03, 0, 0) m
- **origin rpy**: (0, 0, 0.333995) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{9}_{11}(q_{10}) = T_{fixed} \cdot R_{axis}(q_{10})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.94474 & -0.327819 & 0 & 0.03 \\
0.327819 & 0.94474 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{10}) = \begin{bmatrix}
c_{10} & -s_{10} & 0 & 0 \\
s_{10} & c_{10} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2})\quad (L_0 \to L_{2}: \text{Arm01})$$

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3})\quad (L_0 \to L_{3}: \text{Arm02})$$

$$T^{0}_{4} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4})\quad (L_0 \to L_{4}: \text{for_arm_group})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5})\quad (L_0 \to L_{5}: \text{Part02})$$

$$T^{0}_{6} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6}\quad (L_0 \to L_{6}: \text{led_base})$$

$$T^{0}_{7} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6} \cdot T^{6}_{7}(q_{6})\quad (L_0 \to L_{7}: \text{servo_mount})$$

$$T^{0}_{8} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6} \cdot T^{6}_{7}(q_{6}) \cdot T^{7}_{8}(q_{7})\quad (L_0 \to L_{8}: \text{right_pivot})$$

$$T^{0}_{9} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6} \cdot T^{6}_{7}(q_{6}) \cdot T^{7}_{9}(q_{8})\quad (L_0 \to L_{9}: \text{left_pivot})$$

$$T^{0}_{10} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6} \cdot T^{6}_{7}(q_{6}) \cdot T^{7}_{8}(q_{7}) \cdot T^{8}_{10}(q_{9})\quad (L_0 \to L_{10}: \text{acc_right_gripper})$$

$$T^{0}_{11} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6} \cdot T^{6}_{7}(q_{6}) \cdot T^{7}_{9}(q_{8}) \cdot T^{9}_{11}(q_{10})\quad (L_0 \to L_{11}: \text{acc_left_gripper})$$

