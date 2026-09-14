# Research Direction — MPC + Event Trigger + PID

## North star

Mục tiêu của dự án không phải chứng minh MPC luôn tốt hơn PID. Mục tiêu là xác định **khi nào prediction đáng giá hơn chi phí tính toán**, và xây một kiến trúc điều khiển có thể chuyển dần từ mô phỏng sang UAV/UGV/USV và phần cứng thật.

## Kiến trúc nghiên cứu ưu tiên

```text
Mission / Planner
       ↓
State Estimator
       ↓
Prediction + Optimization
       ↓
Event Trigger / Scheduler
       ↓
Predictive reference shaping
       ↓
Fast PID / LQR inner loop
       ↓
Actuator / Plant
       ↓
Sensors
```

MPC là lớp dự đoán và tối ưu. PID là lớp phản xạ nhanh. Event Trigger là lớp quyết định **có đáng trả chi phí solve MPC ở thời điểm này hay không**.

## Thứ tự phát triển bắt buộc

### Gate 1 — Linear baseline

PASS khi:
- PID chạy ổn định trên plant danh định.
- State-space model được hiển thị và thay đổi được.
- Periodic MPC tối ưu control sequence, không phải chỉ một input cố định.
- Input bounds được áp dụng trong solver.
- Disturbance tạo được model prediction error.
- Có metrics chất lượng điều khiển và compute cost.

Không đi tiếp nếu simulation xuất hiện NaN, divergence không giải thích được hoặc solver behavior không lặp lại.

### Gate 2 — Event-triggered predictive guidance

PASS khi:
- Hybrid giữ closed-loop ổn định.
- Watchdog bảo đảm không dùng prediction vô thời hạn.
- Trigger có ít nhất: prediction error, state change, constraint proximity, watchdog.
- Số lần MPC solve thấp hơn periodic MPC đáng kể trong trạng thái bình thường.
- Giảm compute không đổi lấy suy giảm tracking quá mức.

Metric chính:

```text
J_total = w1 * tracking_error
        + w2 * control_effort
        + w3 * solver_time
        + w4 * constraint_violation
```

### Gate 3 — QP formulation

Không gọi solver hiện tại là industrial QP solver.

PASS khi:
- Prediction matrices được xây rõ ràng.
- Cost được đưa về dạng:

  0.5 Uᵀ H U + fᵀ U

- Bounds và rate constraints được biểu diễn độc lập.
- Solver interface tách khỏi controller.
- Có benchmark projected-gradient vs QP backend.
- Ghi lại convergence, iterations, solve time, infeasibility.

### Gate 4 — Estimation

PASS khi:
- Measurement noise được mô phỏng.
- Observer/Kalman Filter cải thiện state estimate.
- Controller không còn giả định đo trực tiếp mọi state.

Sau đó mới đi EKF/UKF.

### Gate 5 — Nonlinear plants

Thứ tự đề xuất:
1. UGV bicycle model — dễ kiểm chứng geometry và trajectory tracking.
2. UAV planar/attitude model — tăng coupling và constraint complexity.
3. USV planar model — thêm disturbance chậm, drag và current/wind.

Không nhảy ngay vào full 6-DOF UAV trước khi planar models PASS.

### Gate 6 — NMPC

PASS khi:
- Nonlinear model rõ ràng.
- Có warm start.
- Có timing budget.
- Có fallback controller khi solver fail hoặc timeout.
- Constraint handling được kiểm thử bằng scenario cực đoan.

### Gate 7 — Adaptive / Learning MPC

Chỉ bắt đầu sau khi classical baseline đã PASS.

AI/ML được dùng trước cho:
- residual dynamics,
- disturbance estimation,
- online parameter identification,
- adaptive trigger threshold.

Không cho neural network thay toàn bộ safety-critical control loop ở giai đoạn đầu.

### Gate 8 — Hardware

PASS khi:
- Controller có fixed timing budget.
- Có telemetry log.
- Có HIL/SIL comparison.
- Có deadline miss counter.
- Có fallback PID/LQR.
- Có export config và reproducible experiment preset.

## Quy tắc không đi lạc hướng

1. Mỗi tính năng mới phải trả lời một câu hỏi nghiên cứu cụ thể.
2. Mỗi controller mới phải có baseline để so sánh.
3. Mỗi tối ưu compute phải đo cả ảnh hưởng lên chất lượng điều khiển.
4. Không thêm AI chỉ để dự án trông hiện đại hơn.
5. Không tuyên bố real-time nếu chưa benchmark trên target hardware.
6. Không tối ưu giao diện trước khi core algorithm có test.
7. Không dùng một plant duy nhất để kết luận controller tốt hơn tổng quát.
8. Mọi experiment quan trọng phải tái lập được bằng preset/config.

## Milestone kế tiếp

**V0.3 — QP-ready MPC core**

- prediction matrix builder,
- condensed quadratic cost,
- solver adapter interface,
- Δu and hard input constraints,
- convergence diagnostics,
- experiment presets,
- baseline regression tests.

Chỉ sau V0.3 mới bắt đầu Kalman Filter và plant models chuyên biệt.
