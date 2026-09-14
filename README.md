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

## V0.3 hiện tại — QP Core

- React + Vite research web-app.
- Plant rời rạc dạng state-space với trạng thái `[position, velocity]ᵀ`.
- PID fast loop có saturation và anti-windup cơ bản.
- MPC tối ưu **control sequence** trên finite horizon.
- Condensed prediction model:

```text
X = Φ x₀ + Γ U
```

- Quadratic objective ở dạng:

```text
min  0.5 Uᵀ H U + fᵀ U
```

- Hai solver backend:
  - `box-qp`: solver chính cho convex QP với hard input bounds.
  - `projected-gradient`: baseline/fallback để benchmark.
- Box-QP dùng warm-start, accelerated projected steps và monotone restart.
- Diagnostics cho từng lần solve:
  - convergence status,
  - iteration count,
  - KKT residual,
  - feasibility violation,
  - active lower/upper bounds,
  - active constraint ratio,
  - solver time.
- Hybrid MPC → PID dùng **predictive reference shaping**: MPC dự đoán sai số tương lai rồi tạo setpoint dẫn trước có giới hạn cho PID.
- Event trigger dựa trên model prediction error, normalized state change, actuator constraint proximity và watchdog timeout.
- Disturbance injection chỉ tác động lên plant thật; MPC dùng model danh định.
- Dashboard hiển thị response, trigger timeline, compute reduction và solver-health metrics.
- Smoke test kiểm tra:
  - state/control hữu hạn,
  - event-trigger giảm số lần solve,
  - condensed QP khớp rollout objective,
  - Hessian đối xứng,
  - hard input bounds,
  - feasibility,
  - KKT residual.
- GitHub Actions CI chạy control-core smoke test + production build.

> V0.3 hiện đã có **QP backend thực cho bài toán convex có box input constraints**, nhưng chưa phải generic industrial QP stack. Hard slew-rate constraints `Δu`, state/output inequalities và OSQP/WASM backend vẫn là các gate tiếp theo.

## Cấu trúc lõi

```text
src/core/
├── models/
│   └── secondOrderPlant.js
├── controllers/
│   └── pid.js
├── mpc/
│   ├── condensedQP.js
│   └── rollout.js
├── solvers/
│   ├── index.js
│   ├── boxQPMPC.js
│   └── projectedGradientMPC.js
├── triggers/
│   └── eventTrigger.js
└── simulator.js
```

`simulator.js` chỉ đóng vai trò orchestration. Plant, controller, QP formulation, solver và trigger policy được tách độc lập để sau này thay backend, thêm Kalman Filter hoặc chuyển sang UAV/UGV/USV mà không phải viết lại toàn bộ hệ thống.

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
- `H`, `f` và input bounds.
- Solver adapter.
- Box-QP backend.
- KKT/feasibility/convergence diagnostics.
- Regression test giữa condensed objective và rollout objective.

### Gate B2 — General constrained MPC — NEXT
- Hard `Δu` / slew-rate constraints.
- State/output inequalities.
- Explicit infeasibility handling.
- Benchmark box-QP vs external QP backend.
- Reproducible experiment presets.

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

1. PID không bị loại bỏ; PID giữ vòng phản xạ nhanh khi nó là lựa chọn phù hợp.
2. MPC chỉ được dùng ở nơi prediction, multivariable coupling hoặc constraints mang lại giá trị.
3. MPC không bắt buộc solve theo timer cố định; Event Trigger phải chứng minh được lợi ích bằng số liệu.
4. Watchdog luôn tồn tại để tránh dùng prediction quá cũ.
5. Mọi controller phải được đánh giá theo **control quality + compute cost + solver quality**.
6. Simulation core, plant model, solver, trigger policy và UI phải tách lớp.
7. Không chuyển sang AI/Learning MPC trước khi baseline classical control được kiểm chứng.
8. Không gọi một thuật toán là “real-time” nếu chưa có timing benchmark trên target hardware.

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
