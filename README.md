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

## V0.3B hiện tại — General Constrained MPC

- React + Vite research web-app.
- Plant rời rạc dạng state-space với trạng thái `[position, velocity]ᵀ`.
- PID fast loop có saturation và anti-windup cơ bản.
- MPC tối ưu **control sequence** trên finite horizon.
- Condensed prediction model:

```text
X = Φ x₀ + Γ U
```

- Quadratic objective:

```text
min  0.5 Uᵀ H U + fᵀ U
```

- Constraint representation thống nhất:

```text
A U <= b
```

bao gồm:
- hard input bounds `uMin <= u(k) <= uMax`,
- hard slew-rate bounds `ΔuMin <= u(k)-u(k-1) <= ΔuMax`.

- Ba solver backend:
  - `constrained-qp`: solver mặc định cho polyhedral input/rate constraints,
  - `box-qp`: baseline QP cho hard input bounds,
  - `projected-gradient`: legacy research baseline.
- `constrained-qp` dùng warm-start, accelerated projected gradient và sparse Dykstra projection lên giao các half-space.
- Explicit solver status:
  - `solved`,
  - `max-iterations` nhưng feasible/approximate,
  - `timeout`,
  - `infeasible`,
  - `numerical-failure`.
- Fallback semantics: timeout/infeasible/numerical failure không được âm thầm tạo guidance mới cho hybrid PID.
- Diagnostics cho từng lần solve:
  - convergence status,
  - iteration count,
  - projected-gradient/KKT-style stationarity residual,
  - feasibility violation,
  - active input/rate inequalities,
  - projection cycles,
  - solver time,
  - fallback status.
- Hybrid MPC → PID dùng **predictive reference shaping**.
- Event trigger dựa trên model prediction error, normalized state change, actuator constraint proximity và watchdog timeout.
- Disturbance injection chỉ tác động lên plant thật; MPC dùng model danh định.
- Dashboard hiển thị response, trigger timeline, compute reduction, hard constraints và solver-health metrics.
- Smoke test kiểm tra:
  - state/control hữu hạn,
  - event-trigger giảm số lần solve,
  - condensed QP khớp rollout objective,
  - Hessian đối xứng,
  - hard input bounds,
  - hard `Δu`,
  - independent `AU<=b` feasibility check,
  - explicit infeasibility/fallback semantics.
- GitHub Actions CI chạy control-core smoke test + production build.

> V0.3B đã vượt khỏi box-only MPC. Bước kế tiếp không phải Kalman ngay: cần hoàn thiện **state/output inequalities**, reproducible experiment presets và benchmark backend trước khi đóng toàn bộ Gate QP.

## Cấu trúc lõi

```text
src/core/
├── models/
│   └── secondOrderPlant.js
├── controllers/
│   └── pid.js
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

`simulator.js` chỉ đóng vai trò orchestration. Plant, controller, QP formulation, constraints, solver và trigger policy được tách độc lập để sau này thay backend, thêm Kalman Filter hoặc chuyển sang UAV/UGV/USV mà không viết lại toàn bộ hệ thống.

## Research gates

### Gate A — Linear research core — PASS
- State-space plant.
- PID benchmark.
- MPC sequence optimization.
- Event-trigger + watchdog.
- Disturbance response.
- Compute profiling.

### Gate B1 — Condensed QP + box constraints — PASS
- Prediction matrices `Φ`, `Γ`.
- `H`, `f` và hard input bounds.
- Solver adapter.
- Box-QP backend.
- Optimality/feasibility diagnostics.
- Regression test condensed objective ↔ rollout objective.

### Gate B2 — General constrained MPC — PARTIAL PASS
Đã có:
- generic `AU <= b` representation,
- hard `Δu` / slew-rate constraints,
- sparse polyhedral projection,
- explicit timeout/infeasible/numerical-failure status,
- safe fallback semantics.

Còn thiếu trước khi đóng Gate B2:
- state/output inequalities,
- reproducible experiment presets,
- backend benchmark suite,
- deadline/failure scenario regression tests mở rộng.

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

Chi tiết định hướng và tiêu chí PASS/FAIL nằm trong [`docs/RESEARCH_DIRECTION.md`](docs/RESEARCH_DIRECTION.md).

## Nguyên tắc kiến trúc

1. PID không bị loại bỏ; PID giữ vòng phản xạ nhanh khi phù hợp.
2. MPC chỉ dùng nơi prediction, coupling hoặc constraints tạo giá trị thực.
3. Event Trigger phải chứng minh lợi ích bằng số liệu.
4. Watchdog luôn tồn tại để tránh dùng prediction quá cũ.
5. Mọi controller được đánh giá theo **control quality + compute cost + solver quality + feasibility**.
6. Không phát guidance mới từ nghiệm timeout/infeasible/numerically invalid.
7. Core, plant model, constraints, solver, trigger policy và UI phải tách lớp.
8. Không chuyển sang AI/Learning MPC trước khi baseline classical control được kiểm chứng.
9. Không gọi thuật toán là “real-time” nếu chưa benchmark trên target hardware.

## Chạy local

```bash
npm install
npm run smoke
npm run dev
```

Build production:

```bash
npm run build
```
