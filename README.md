# MPC_PID_System

Web-app nghiên cứu và mô phỏng kiến trúc điều khiển lai **MPC/NMPC + Event Trigger + PID**.

## Mục tiêu

Không xây một dashboard minh họa đơn giản. Dự án hướng tới một **Control Research Workbench** có thể dùng để học, thiết kế, mô phỏng, so sánh và dần tiến tới triển khai thuật toán trên phần cứng thật.

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
    MPC / NMPC         │ reuse previous prediction
        ↓              │
 Optimal reference  ←──┘
        ↓
 Fast PID / LQR loop
        ↓
 Actuator / Plant
        ↓
      Sensor
```

## V0.1 hiện tại

- React + Vite web-app.
- Mô hình plant rời rạc bậc hai đơn giản.
- PID controller độc lập.
- MPC finite-horizon educational solver.
- Hybrid MPC → PID.
- Event trigger dựa trên prediction error, state error và watchdog timeout.
- So sánh trực tiếp PID / MPC / Event-triggered MPC+PID.
- Metrics: overshoot, settling time, IAE, control effort, MPC solve count.
- Điều chỉnh Kp/Ki/Kd, horizon, Q/R và trigger thresholds trực tiếp trên giao diện.

> V0.1 dùng solver dự đoán dạng educational/grid-search để làm rõ logic hệ thống. Giai đoạn sau sẽ thay bằng QP/NMPC solver đúng chuẩn và benchmark thời gian tính toán.

## Roadmap

### Phase 1 — Research Core
1. PID/P/PI/PID laboratory.
2. State-space plant models.
3. Discrete MPC with true quadratic optimization.
4. Event-triggered MPC.
5. Hybrid MPC + PID supervisor.
6. Disturbance injection and robustness tests.
7. Experiment save/load and reproducible presets.

### Phase 2 — Modern Control
8. LQR/LQI comparison.
9. Observer, Kalman Filter, EKF, UKF.
10. Constraints: state/input/rate limits.
11. Adaptive trigger threshold.
12. Compute profiler and solver benchmark.
13. Stability and feasibility diagnostics.

### Phase 3 — Nonlinear / Autonomous Systems
14. NMPC.
15. UAV model.
16. UGV bicycle model.
17. USV planar model.
18. Trajectory tracking.
19. Wind/current/slope disturbances.
20. Obstacle/constraint-aware control.

### Phase 4 — Intelligent Control
21. System Identification.
22. Online model adaptation.
23. Learning-based residual model.
24. Adaptive/Learning MPC.
25. AI experiment assistant and automatic controller comparison.

### Phase 5 — Hardware path
26. Export controller parameters/configuration.
27. Hardware-in-the-loop interface.
28. STM32/ESP32/SoC timing benchmark.
29. Telemetry stream input.
30. Real plant validation.

## Nguyên tắc kiến trúc

- PID không bị loại bỏ; PID là vòng phản xạ nhanh.
- MPC không bắt buộc solve theo timer cố định.
- Event Trigger quyết định khi nào cần tái dự đoán/tối ưu.
- Luôn có watchdog timeout để tránh dùng prediction quá cũ.
- Mỗi controller phải đo được cả chất lượng điều khiển và chi phí tính toán.
- Simulation core tách khỏi UI để sau này có thể thay plant/solver mà không viết lại giao diện.

## Chạy local

```bash
npm install
npm run dev
```

Build production:

```bash
npm run build
```
