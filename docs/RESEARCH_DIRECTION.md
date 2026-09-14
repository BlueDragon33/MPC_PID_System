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
- Ghi lại convergence, iterations, solve time, optimality residual, feasibility violation và active constraints.
- Dashboard hiển thị solver health thay vì chỉ hiển thị thời gian solve.

### Gate 3B — General constrained MPC — PARTIAL PASS

Đã hoàn thành:

- hard slew-rate constraints:

```text
Δu_min ≤ u_k - u_{k-1} ≤ Δu_max
```

- generalized inequality representation:

```text
A U ≤ b
```

- hard input bounds và hard `Δu` dùng chung constraint layer,
- sparse Dykstra projection lên giao các half-space,
- projected-gradient stationarity residual cho polyhedral feasible set,
- explicit solver status:
  - `solved`,
  - `max-iterations` nhưng feasible/approximate,
  - `timeout`,
  - `infeasible`,
  - `numerical-failure`,
- fallback policy: timeout/infeasible/numerical failure không được tạo guidance mới cho hybrid PID,
- regression test độc lập cho `AU≤b` và hard `Δu`.

Gate 3B **chưa đóng** cho đến khi có đủ:

- state/output inequalities,
- reproducible experiment presets,
- constraint activation scenarios mở rộng,
- backend benchmark suite,
- external QP backend benchmark (ưu tiên OSQP/WASM hoặc backend tương đương khi thực sự cần general constraints lớn hơn),
- deadline/failure regression scenarios.

Không bắt đầu Kalman Filter trước khi state/output constraints và reproducible experiments đủ để khóa Gate 3B.

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
11. Constraint penalty trong objective không được nhầm với hard constraint; hai khái niệm phải được thể hiện riêng trong code và UI.

## Milestone kế tiếp

**V0.3C — State/output constraints + reproducible experiments**

Thứ tự thực hiện:

1. state/output inequality builder từ `Φ`, `Γ`,
2. hard position/velocity constraints,
3. constraint activation scenarios,
4. experiment preset schema + save/load/export,
5. solver benchmark suite,
6. timeout/infeasible regression scenarios,
7. đánh giá có cần external QP/WASM backend ở quy mô hiện tại hay chưa.

Chỉ sau khi V0.3C PASS mới mở Gate 4 — State Estimation.
