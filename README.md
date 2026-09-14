# MPC_PID_System

Web-app nghiên cứu và mô phỏng kiến trúc điều khiển lai **MPC/NMPC + Event Trigger + PID**.

## Mục tiêu

Dự án không dừng ở dashboard minh họa. Đích đến là một **Control Research Workbench** dùng để học, thiết kế, mô phỏng, benchmark và dần tiến tới triển khai trên phần cứng thật.

Flow cốt lõi:

```text
Reference / Planner
        ↓
 State Estimator
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
 Actuator / Plant
        ↓
      Sensor
```

## V0.3C hiện tại — Predicted Safety Envelope + Reproducible Experiments

Lõi MPC đã chuyển từ box-only sang general constrained research core.

### Prediction + optimization

```text
X = Φ x₀ + Γ U
```

```text
min  0.5 Uᵀ H U + fᵀ U
s.t. A U <= b
```

Constraint polyhedron hiện hỗ trợ:

- hard input bounds,
- hard slew-rate / `Δu`,
- predicted position bounds,
- predicted velocity bounds,
- predicted output bounds.

State/output inequalities được sinh trực tiếp từ `Φ`, `Γ` và free response, không nhét thủ công vào controller.

### Solver stack

Ba backend được giữ để nghiên cứu chéo:

- `constrained-qp`: backend chính cho general `AU<=b`,
- `box-qp`: QP baseline cho input box,
- `projected-gradient`: sequence-optimization baseline.

`constrained-qp` có:

- warm start,
- accelerated projected gradient,
- sparse Dykstra projection,
- adaptive projection refinement khi state/output constraints được bật,
- explicit status: `solved`, `max-iterations`, `timeout`, `infeasible`, `numerical-failure`,
- feasibility + stationarity diagnostics,
- active-constraint statistics,
- safe fallback semantics.

Khi state/output envelope bất khả thi, fallback ưu tiên giữ **actuator magnitude + slew-rate constraints** hợp lệ và báo rõ envelope nào không thể thỏa. Nó không cố “cứu” state constraint bằng cách phát lệnh actuator phi vật lý.

### Model safety vs real plant safety

V0.3C tách hai khái niệm:

1. **QP/model feasibility** — quỹ đạo MPC dự đoán có thỏa `AU<=b` hay không.
2. **Actual plant safety** — plant thật sau PID + disturbance có thực sự còn trong envelope hay không.

Điểm này đặc biệt quan trọng cho hybrid guidance-only: MPC tạo reference cho PID nhưng PID vẫn là execution loop riêng. Do đó predicted feasibility chưa tự động đồng nghĩa với hard safety guarantee của plant thật.

Dashboard hiển thị riêng:

- max QP feasibility violation,
- max actual safety violation,
- unsafe sample rate,
- fraction of solves touching state/output boundary,
- fallback / timeout / infeasible counts.

### Reproducible experiment system

Có các preset:

- `baseline`,
- `rate-limited`,
- `safety-envelope`,
- `disturbance-stress`,
- `infeasible-guard`.

Experiment dùng schema versioned:

```text
mpc-pid-experiment/v1
```

Web-app hỗ trợ:

- Save local,
- Load local,
- Export JSON,
- Import JSON.

Điều này cho phép một scenario được chạy lại đúng cấu hình thay vì phụ thuộc thao tác tay.

### Regression + benchmark

Smoke test kiểm tra:

- state/control hữu hạn,
- event-trigger giảm số lần solve,
- condensed objective ↔ rollout objective,
- Hessian symmetry,
- hard input bounds,
- hard `Δu`,
- independent `AU<=b` feasibility,
- predicted state/output envelope,
- impossible-envelope infeasibility,
- actuator-safe fallback.

`npm run benchmark` so sánh solver, periodic MPC và hybrid theo:

- IAE,
- solve count,
- average/max solve time,
- convergence,
- fallback,
- QP feasibility violation,
- actual plant safety violation.

CI lưu cả `smoke.log` và `benchmark.log` làm research artifacts.

## Cấu trúc lõi

```text
src/core/
├── models/
│   └── secondOrderPlant.js
├── controllers/
│   └── pid.js
├── experiments/
│   ├── presets.js
│   └── serialization.js
├── mpc/
│   ├── condensedQP.js
│   ├── constraints.js
│   └── rollout.js
├── solvers/
│   ├── index.js
│   ├── constrainedQPMPC.js
│   ├── boxQPMPC.js
│   └── projectedGradientMPC.js
├── triggers/
│   └── eventTrigger.js
└── simulator.js
```

`simulator.js` chỉ orchestration. Plant, controller, QP formulation, constraints, solver, experiment schema và trigger policy được tách độc lập.

## Research gates

### Gate A — Linear research core — PASS
- State-space plant.
- PID benchmark.
- MPC sequence optimization.
- Event-trigger + watchdog.
- Disturbance response.
- Compute profiling.

### Gate B1 — Condensed QP + box constraints — PASS
- `Φ`, `Γ`, `H`, `f`.
- Hard input bounds.
- Solver adapter.
- QP optimality/feasibility diagnostics.

### Gate B2 — General input/rate constrained MPC — PASS
- Generic `AU<=b`.
- Hard `Δu`.
- Explicit infeasibility/failure semantics.
- Actuator-safe fallback.

### Gate B3 — Predicted state/output safety + reproducible experiments — ACTIVE
Đã có:
- predicted position/velocity/output inequalities,
- safety-envelope scenarios,
- versioned experiment JSON,
- save/load/import/export,
- internal solver + safety benchmark,
- real-plant safety audit.

Còn quyết định research gate kế tiếp:
- nếu hybrid guidance-only cho thấy plant-safety gap đáng kể, thêm **Safety Governor / admissibility filter** trước State Estimation,
- external QP/WASM backend chỉ thêm khi benchmark chứng minh backend hiện tại là bottleneck hoặc constraint scale đòi hỏi.

### Gate C — State estimation
- Observer / Kalman Filter.
- Measurement noise.
- Model mismatch.
- EKF / UKF khi chuyển nonlinear.

### Gate D — Autonomous plant models
- UGV bicycle model.
- UAV attitude/position model.
- USV planar model.
- Trajectory tracking + constraints + disturbances.

### Gate E — NMPC and intelligence
- Nonlinear prediction.
- NMPC solver.
- Online system identification.
- Adaptive trigger threshold.
- Learning residual dynamics / Learning MPC.

### Gate F — Hardware path
- HIL interface.
- Telemetry ingestion.
- Timing benchmark trên STM32/ESP32/SoC.
- Export controller configuration.
- Real plant validation.

Chi tiết gate nằm trong [`docs/RESEARCH_DIRECTION.md`](docs/RESEARCH_DIRECTION.md).

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
