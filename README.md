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

### Reproducible experiments

Preset hiện có gồm:

- `baseline`,
- `rate-limited`,
- `safety-envelope`,
- `disturbance-stress`,
- `noisy-estimation`,
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
- **Model mismatch + disturbance-state estimation — ACTIVE**
- Nonlinear UGV/UAV/USV models — NEXT
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
