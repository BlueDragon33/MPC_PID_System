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

### Gate 1 — Linear baseline — PASS

- PID chạy ổn định trên plant danh định.
- State-space model được hiển thị và thay đổi được.
- Periodic MPC tối ưu control sequence, không phải chỉ một input cố định.
- Input bounds được áp dụng trong solver.
- Disturbance tạo được model prediction error.
- Có metrics chất lượng điều khiển và compute cost.

### Gate 2 — Event-triggered predictive guidance — PASS

- Hybrid giữ closed-loop ổn định.
- Watchdog bảo đảm không dùng prediction vô thời hạn.
- Trigger gồm prediction error, state change, constraint proximity và watchdog.
- Số lần MPC solve thấp hơn periodic MPC đáng kể trong trạng thái bình thường.
- Compute saving được đo cùng tracking quality.

Metric tổng quát:

```text
J_total = w1 * tracking_error
        + w2 * control_effort
        + w3 * solver_time
        + w4 * constraint_violation
```

### Gate 3A — Condensed QP + box-constrained MPC — PASS

- Prediction matrices `Φ`, `Γ` được xây rõ ràng.
- Cost được đưa về:

```text
0.5 Uᵀ H U + fᵀ U
```

- Condensed objective được regression-test trực tiếp với rollout objective.
- Solver interface tách khỏi controller/simulator.
- Có `box-qp` backend và projected-gradient baseline.
- Hard input bounds được giữ khả thi bằng projection.
- Ghi lại:
  - convergence,
  - iterations,
  - solve time,
  - KKT residual,
  - feasibility violation,
  - active constraints.
- Dashboard hiển thị solver health thay vì chỉ hiển thị thời gian solve.

### Gate 3B — General constrained MPC — NEXT

Đây là gate kế tiếp, chưa được coi là PASS cho đến khi có đủ:

- hard slew-rate constraints:

```text
Δu_min ≤ u_k - u_{k-1} ≤ Δu_max
```

- state/output inequalities,
- explicit feasible/infeasible status,
- fallback behavior khi solver không hội tụ hoặc bài toán infeasible,
- benchmark nội bộ giữa box-QP và một external QP backend (ưu tiên OSQP/WASM hoặc backend tương đương),
- reproducible experiment presets,
- regression scenarios cho constraint activation.

Không bắt đầu Kalman Filter trước khi ít nhất hard `Δu` constraints và solver-failure semantics được hoàn thiện.

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
9. Một solver chỉ được coi là tốt khi **objective + feasibility + optimality + timing** cùng đạt yêu cầu.
10. Khi solver fail, hệ thống phải có semantics rõ ràng; không được âm thầm dùng một nghiệm không đạt chuẩn.

## Milestone kế tiếp

**V0.3B — General constrained MPC**

Thứ tự thực hiện:

1. hard `Δu` constraints,
2. generalized inequality representation,
3. feasible/infeasible solver status,
4. fallback policy,
5. constraint activation scenarios,
6. experiment presets,
7. external QP benchmark.

Sau V0.3B mới mở Gate 4 — State Estimation.
