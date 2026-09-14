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
 Optimal reference  ←──┘
        ↓
 Fast PID / LQR loop
        ↓
 Actuator / Plant
        ↓
      Sensor
```

## V0.2 hiện tại

- React + Vite web-app.
- Plant rời rạc dạng state-space với trạng thái `[position, velocity]ᵀ`.
- PID fast loop có saturation và anti-windup cơ bản.
- Finite-horizon MPC tối ưu **control sequence** thay vì một lệnh điều khiển cố định.
- Projected-gradient optimizer với input bounds, state cost, input cost và Δu cost.
- Warm-start cho chuỗi điều khiển giữa các lần solve.
- Hybrid MPC → PID supervisor.
- Event trigger dựa trên:
  - model prediction error,
  - normalized state change,
  - actuator constraint proximity,
  - watchdog timeout.
- Disturbance injection chỉ tác động lên plant thật; MPC dùng model danh định để tạo sai lệch có ý nghĩa vật lý.
- Trigger timeline trực quan.
- Compute profiler: solve count, solve rate, average/max/total solver time và phần trăm solve được tránh.
- So sánh PID / periodic MPC / event-triggered MPC+PID.
- Metrics: overshoot, settling time, IAE, control effort, MPC solve count.

> Solver V0.2 là bộ tối ưu convex dạng projected-gradient viết trực tiếp cho research prototype. Nó đã tối ưu cả chuỗi điều khiển và hỗ trợ ràng buộc input, nhưng **chưa được coi là QP solver công nghiệp**. QP backend chuẩn và benchmark solver độc lập là gate kế tiếp.

## Research gates

Dự án chỉ tiến sang tầng tiếp theo khi tầng trước đo được và kiểm chứng được.

### Gate A — Linear research core
- State-space plant đúng và có thể thay model.
- PID benchmark ổn định.
- MPC control-sequence optimization.
- Event-trigger logic có watchdog.
- Disturbance response.
- Compute profiling.

### Gate B — QP MPC
- Viết condensed prediction model.
- Xây `H`, `f`, bounds cho quadratic program.
- Tách solver interface khỏi controller.
- So sánh projected-gradient với QP backend.
- Kiểm tra feasibility, convergence và timing.

### Gate C — State estimation
- Observer / Kalman Filter.
- Measurement noise.
- Model mismatch.
- EKF / UKF khi chuyển nonlinear.

### Gate D — Autonomous plant models
- UAV attitude/position model.
- UGV bicycle model.
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

## Nguyên tắc kiến trúc

1. PID không bị loại bỏ; PID giữ vòng phản xạ nhanh khi nó là lựa chọn phù hợp.
2. MPC chỉ được dùng ở nơi prediction, multivariable coupling hoặc constraints mang lại giá trị.
3. MPC không bắt buộc solve theo timer cố định; Event Trigger phải chứng minh được lợi ích bằng số liệu.
4. Watchdog luôn tồn tại để tránh dùng prediction quá cũ.
5. Mọi controller phải được đánh giá theo **control quality + compute cost**.
6. Simulation core, plant model, solver, trigger policy và UI phải tách lớp.
7. Không chuyển sang AI/Learning MPC trước khi baseline classical control được kiểm chứng.
8. Không gọi một thuật toán là “real-time” nếu chưa có timing benchmark trên target hardware.

## Chạy local

```bash
npm install
npm run dev
```

Build production:

```bash
npm run build
```
