# MPC_PID_System

Web-app nghiên cứu và mô phỏng kiến trúc điều khiển lai **MPC/NMPC + Event Trigger + PID + State Estimation + Safety Governor**.

## Mục tiêu

Dự án là một **Control Research Workbench** để học, thiết kế, mô phỏng, benchmark và tiến dần tới triển khai trên UAV/UGV/USV và phần cứng thật.

Flow cốt lõi:

```text
Reference / Planner
        ↓
      Sensor
        ↓
 State / disturbance estimator
        ↓
 Prediction Monitor
        ↓
 Event Trigger ────────┐
        ↓              │
    MPC / NMPC         │ reuse previous plan
        ↓              │
 Predictive guidance ←─┘
        ↓
 Fast PID / LQR loop
        ↓
 Safety Governor
        ↓
 Actuator / Plant
```

## Trạng thái hiện tại

### Constrained MPC

```text
X = Φ x₀ + Γ U
```

```text
min  0.5 Uᵀ H U + fᵀ U
s.t. A U <= b
```

Constraint stack hỗ trợ:

- hard input bounds,
- hard slew-rate / `Δu`,
- predicted position bounds,
- predicted velocity bounds,
- predicted output bounds.

Ba solver backend được giữ để nghiên cứu chéo:

- `constrained-qp` — backend chính cho general `AU<=b`,
- `box-qp` — QP baseline cho input box,
- `projected-gradient` — sequence-optimization baseline.

### Event-triggered predictive guidance

MPC không solve ở mọi sample. Event Trigger quyết định khi nào prediction cũ không còn đủ tốt; giữa các lần solve hệ reuse phần MPC plan còn lại.

PID vẫn là fast feedback loop và nhận predictive reference từ MPC.

### Safety Governor

Guidance-only không được xem là hard safety guarantee. Governor kiểm tra PID first move và dùng MPC continuation plan để xây short-horizon admissible command interval.

Benchmark đại diện:

```text
Hybrid guidance-only plant violation: > 0
Hybrid + Safety Governor violation:    0
Safety intervention rate:              ~2.5%
```

Actuator/rate/plan conditioning được đo riêng khỏi state/output safety intervention.

### Linear Kalman state estimation

Khi estimation bật:

- sensor sinh measurement noisy theo seed cố định,
- Kalman Filter tạo `x_hat`, `v_hat` và covariance `P`,
- Event Trigger, MPC, PID và Safety Governor đều dùng estimated state,
- ground truth chỉ dùng để mô phỏng plant và audit.

Kết quả regression đại diện:

```text
measurement RMSE: 0.0986
x-hat RMSE:       0.0546
v-hat RMSE:       0.2161
MPC convergence:  100%
fallback:          0
plant violation:   0
```

### Covariance-aware safety tightening

```text
Δx = kσ sqrt(Pxx)
Δv = kσ sqrt(Pvv)
Δy = kσ sqrt(C P Cᵀ)
```

MPC/Governor dùng envelope đã co để bù uncertainty của estimator. Actual plant safety vẫn được audit bằng envelope gốc.

Sensitivity benchmark hiện chọn `kσ = 1.5` cho preset noisy-estimation vì 2σ trở lên bắt đầu làm solver fallback trong scenario hiện tại.


### Model mismatch + disturbance-state estimation

Truth plant có thể lệch stiffness/damping/gain so với controller model. Preset `mismatch-observer` dùng augmented `x-v-d` Kalman observer với disturbance retention `ρd = 0.90`.

Gate 4B closeout đại diện:

```text
2-state KF:
  IAE 1.6244 · solves 28 · xRMSE 0.0383 · vRMSE 0.1674

x-v-d observer:
  IAE 1.6278 · solves 29 · xRMSE 0.0368 · vRMSE 0.1606
  dRMSE 0.4673 · convergence 100% · fallback 0 · plant violation 0

d-hat-aware Event Monitor:
  prediction triggers 3 → 1
  solves              30 → 29
```

Affine disturbance compensation đã được regression-test trong condensed QP và closed loop, nhưng vẫn **opt-in** vì lợi ích tracking/compute hiện còn rất nhỏ trên linear plant. Không dùng kết quả này để kết luận tổng quát cho nonlinear plants.

### Nonlinear UGV bicycle baseline

Gate 5A now includes a nonlinear 4-state kinematic bicycle plant with a classical Stanley-style lateral controller and PID speed loop.

Representative deterministic regression:

```text
cross-track RMSE: 0.3058 m
heading RMSE:     0.0782 rad
speed RMSE:       0.6415 m/s
unsafe samples:   0
final speed:      4.005 m/s
```

Steering angle, steering-rate, acceleration and speed are bounded explicitly. CI simulation timing is recorded for comparison only and is not treated as a hardware real-time claim.

### Nonlinear UGV EKF estimation

Gate 5B routes the nonlinear UGV controller through a 4-state EKF `[x, y, yaw, v]` with deterministic noisy measurements.

Representative regression:

```text
measurement RMSE x/y/yaw/v:
0.1849 / 0.1762 / 0.03465 / 0.1218

EKF RMSE x/y/yaw/v:
0.03861 / 0.03504 / 0.00835 / 0.03393

unsafe samples: 0
```

Ground truth is reserved for plant propagation and audit; steering/speed control consumes estimated state when Gate 5B estimation is enabled.

### UGV constrained predictive governor

Gate 5C compares the EKF-driven classical UGV controller with a finite-candidate predictive governor using nonlinear bicycle rollouts and explicit constraints.

Representative A/B:

```text
cross-track RMSE: 0.3078 → 0.1287 m
heading RMSE:     0.0787 → 0.0575 rad
control effort:   1.3268 → 1.1802
unsafe samples:   0 → 0
compute/step:     68.6 → 82.0 us
predicted feasible: 100%
fallback:            0
```

This is intentionally **not labeled NMPC**: it does not optimize a full nonlinear control sequence and has no nonlinear-program warm start/solver yet.

### Reproducible experiments

Preset hiện có gồm:

- `baseline`,
- `rate-limited`,
- `safety-envelope`,
- `disturbance-stress`,
- `noisy-estimation`,
- `model-mismatch`,
- `mismatch-observer`,
- `infeasible-guard`.

Experiment dùng schema:

```text
mpc-pid-experiment/v1
```

Web-app hỗ trợ Save/Load local, Export/Import JSON. CI lưu `smoke.log` và `benchmark.log` làm research artifacts.

## Cấu trúc lõi

```text
src/core/
├── models/
│   └── secondOrderPlant.js
├── controllers/
│   └── pid.js
├── estimation/
│   ├── linearKalmanFilter.js
│   ├── measurementSensor.js
│   └── uncertaintyTightening.js
├── experiments/
│   ├── presets.js
│   └── serialization.js
├── mpc/
│   ├── condensedQP.js
│   ├── constraints.js
│   └── rollout.js
├── safety/
│   └── shortHorizonGovernor.js
├── solvers/
│   ├── index.js
│   ├── constrainedQPMPC.js
│   ├── boxQPMPC.js
│   └── projectedGradientMPC.js
├── triggers/
│   └── eventTrigger.js
└── simulator.js
```

## Research gates

- Linear research core — PASS
- Event-triggered predictive guidance — PASS
- Condensed QP + box constraints — PASS
- General input/rate constrained MPC — PASS
- Predicted state/output safety — PASS
- Safety Governor / admissibility filter — PASS
- Linear Kalman state estimation + covariance-aware safety — PASS
- Model mismatch + disturbance-state estimation — PASS
- Nonlinear UGV bicycle + classical baseline — PASS
- UGV nonlinear state estimation — PASS
- UGV constrained predictive control comparison — PASS
- **UAV planar/attitude classical baseline — NEXT**
- USV planar model — LATER
- NMPC — LATER
- Adaptive / Learning MPC — LATER
- HIL / hardware timing validation — LATER

Chi tiết nằm trong [`docs/RESEARCH_DIRECTION.md`](docs/RESEARCH_DIRECTION.md).

## Chạy local

```bash
npm install
npm run smoke
npm run benchmark
npm run dev
```

Build production:

```bash
npm run build
```
