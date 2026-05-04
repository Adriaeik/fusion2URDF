# Transformation Matrices - Panther

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | fr_wheel_base_link |
| $L_{2}$ | fl_wheel_base_link |
| $L_{3}$ | rr_wheel_base_link |
| $L_{4}$ | rl_wheel_base_link |
| $L_{5}$ | cover_link |
| $L_{6}$ | imu_link |
| $L_{7}$ | base_footprint |
| $L_{8}$ | front_bumper_link |
| $L_{9}$ | rear_bumper_link |
| $L_{10}$ | fr_wheel_link |
| $L_{11}$ | fl_wheel_link |
| $L_{12}$ | rr_wheel_link |
| $L_{13}$ | rl_wheel_link |
| $L_{14}$ | mount_link |
| $L_{15}$ | lights_channel_1_link |
| $L_{16}$ | lights_channel_2_link |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | fr_wheel_joint | continuous (rad) | $L_{1}$ | $L_{10}$ |
| $q_{2}$ | fl_wheel_joint | continuous (rad) | $L_{2}$ | $L_{11}$ |
| $q_{3}$ | rr_wheel_joint | continuous (rad) | $L_{3}$ | $L_{12}$ |
| $q_{4}$ | rl_wheel_joint | continuous (rad) | $L_{4}$ | $L_{13}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  |-- [fixed] fr_wheel_base_joint
  |   L1: fr_wheel_base_link
  |     +-- [continuous] fr_wheel_joint (q1)
  |         L10: fr_wheel_link
  |-- [fixed] fl_wheel_base_joint
  |   L2: fl_wheel_base_link
  |     +-- [continuous] fl_wheel_joint (q2)
  |         L11: fl_wheel_link
  |-- [fixed] rr_wheel_base_joint
  |   L3: rr_wheel_base_link
  |     +-- [continuous] rr_wheel_joint (q3)
  |         L12: rr_wheel_link
  |-- [fixed] rl_wheel_base_joint
  |   L4: rl_wheel_base_link
  |     +-- [continuous] rl_wheel_joint (q4)
  |         L13: rl_wheel_link
  |-- [fixed] body_to_cover_joint
  |   L5: cover_link
  |     +-- [fixed] cover_to_mount_joint
  |         L14: mount_link
  |-- [fixed] body_to_imu_joint
  |   L6: imu_link
  |-- [fixed] body_to_footprint_joint
  |   L7: base_footprint
  |-- [fixed] body_to_front_bumper_joint
  |   L8: front_bumper_link
  |     +-- [fixed] front_bumper_to_lights_channel_1_joint
  |         L15: lights_channel_1_link
  +-- [fixed] body_to_rear_bumper_joint
      L9: rear_bumper_link
        +-- [fixed] rear_bumper_to_lights_channel_2_joint
            L16: lights_channel_2_link
```

## Transforms

## fr_wheel_base_joint

$L_{0}$ **base_link** -> $L_{1}$ **fr_wheel_base_link** (fixed)

- **origin xyz**: (0.22, -0.258, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{1} = \begin{bmatrix}
1 & 0 & 0 & 0.22 \\
0 & 1 & 0 & -0.258 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## fl_wheel_base_joint

$L_{0}$ **base_link** -> $L_{2}$ **fl_wheel_base_link** (fixed)

- **origin xyz**: (0.22, 0.258, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{2} = \begin{bmatrix}
1 & 0 & 0 & 0.22 \\
0 & 1 & 0 & 0.258 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## rr_wheel_base_joint

$L_{0}$ **base_link** -> $L_{3}$ **rr_wheel_base_link** (fixed)

- **origin xyz**: (-0.22, -0.258, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{3} = \begin{bmatrix}
1 & 0 & 0 & -0.22 \\
0 & 1 & 0 & -0.258 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## rl_wheel_base_joint

$L_{0}$ **base_link** -> $L_{4}$ **rl_wheel_base_link** (fixed)

- **origin xyz**: (-0.22, 0.258, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{4} = \begin{bmatrix}
1 & 0 & 0 & -0.22 \\
0 & 1 & 0 & 0.258 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## body_to_cover_joint

$L_{0}$ **base_link** -> $L_{5}$ **cover_link** (fixed)

- **origin xyz**: (0, 0, 0.14) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{5} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0.14 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## body_to_imu_joint

$L_{0}$ **base_link** -> $L_{6}$ **imu_link** (fixed)

- **origin xyz**: (0.169, 0.025, 0.092) m
- **origin rpy**: (0, 0, -1.570796) rad

### Local Transform

$$
T^{0}_{6} = \begin{bmatrix}
0 & 1 & 0 & 0.169 \\
-1 & 0 & 0 & 0.025 \\
0 & 0 & 1 & 0.092 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## body_to_footprint_joint

$L_{0}$ **base_link** -> $L_{7}$ **base_footprint** (fixed)

- **origin xyz**: (0, 0, -0.1825) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{7} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & -0.1825 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## body_to_front_bumper_joint

$L_{0}$ **base_link** -> $L_{8}$ **front_bumper_link** (fixed)

- **origin xyz**: (0.362, 0, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{8} = \begin{bmatrix}
1 & 0 & 0 & 0.362 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## body_to_rear_bumper_joint

$L_{0}$ **base_link** -> $L_{9}$ **rear_bumper_link** (fixed)

- **origin xyz**: (-0.362, 0, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{9} = \begin{bmatrix}
1 & 0 & 0 & -0.362 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## fr_wheel_joint

$L_{1}$ **fr_wheel_base_link** -> $L_{10}$ **fr_wheel_link** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0, -0.091, 0) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, -1, 0)

### Local Transform

$$
T^{1}_{10}(q_{1}) = \begin{bmatrix}
c_{1} & 0 & -s_{1} & 0 \\
0 & 1 & 0 & -0.091 \\
s_{1} & 0 & c_{1} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## fl_wheel_joint

$L_{2}$ **fl_wheel_base_link** -> $L_{11}$ **fl_wheel_link** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0, 0.091, 0) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, -1, 0)

### Local Transform

$$
T^{2}_{11}(q_{2}) = \begin{bmatrix}
c_{2} & 0 & -s_{2} & 0 \\
0 & 1 & 0 & 0.091 \\
s_{2} & 0 & c_{2} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## rr_wheel_joint

$L_{3}$ **rr_wheel_base_link** -> $L_{12}$ **rr_wheel_link** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0, -0.091, 0) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, -1, 0)

### Local Transform

$$
T^{3}_{12}(q_{3}) = \begin{bmatrix}
c_{3} & 0 & -s_{3} & 0 \\
0 & 1 & 0 & -0.091 \\
s_{3} & 0 & c_{3} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## rl_wheel_joint

$L_{4}$ **rl_wheel_base_link** -> $L_{13}$ **rl_wheel_link** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (0, 0.091, 0) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, -1, 0)

### Local Transform

$$
T^{4}_{13}(q_{4}) = \begin{bmatrix}
c_{4} & 0 & -s_{4} & 0 \\
0 & 1 & 0 & 0.091 \\
s_{4} & 0 & c_{4} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## cover_to_mount_joint

$L_{5}$ **cover_link** -> $L_{14}$ **mount_link** (fixed)

- **origin xyz**: (0, 0, 0.0315) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{5}_{14} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0.0315 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## front_bumper_to_lights_channel_1_joint

$L_{8}$ **front_bumper_link** -> $L_{15}$ **lights_channel_1_link** (fixed)

- **origin xyz**: (0, 0, 0.0185) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{8}_{15} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0.0185 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## rear_bumper_to_lights_channel_2_joint

$L_{9}$ **rear_bumper_link** -> $L_{16}$ **lights_channel_2_link** (fixed)

- **origin xyz**: (0, 0, 0.0185) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{9}_{16} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0.0185 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{10} = T^{0}_{1} \cdot T^{1}_{10}(q_{1})\quad (L_0 \to L_{10}: \text{fr_wheel_link})$$

$$T^{0}_{11} = T^{0}_{2} \cdot T^{2}_{11}(q_{2})\quad (L_0 \to L_{11}: \text{fl_wheel_link})$$

$$T^{0}_{12} = T^{0}_{3} \cdot T^{3}_{12}(q_{3})\quad (L_0 \to L_{12}: \text{rr_wheel_link})$$

$$T^{0}_{13} = T^{0}_{4} \cdot T^{4}_{13}(q_{4})\quad (L_0 \to L_{13}: \text{rl_wheel_link})$$

$$T^{0}_{14} = T^{0}_{5} \cdot T^{5}_{14}\quad (L_0 \to L_{14}: \text{mount_link})$$

$$T^{0}_{15} = T^{0}_{8} \cdot T^{8}_{15}\quad (L_0 \to L_{15}: \text{lights_channel_1_link})$$

$$T^{0}_{16} = T^{0}_{9} \cdot T^{9}_{16}\quad (L_0 \to L_{16}: \text{lights_channel_2_link})$$

