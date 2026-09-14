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
Safety Authority / Governor   ← decision gate
       ↓
Fast PID / LQR inner loop
       ↓
Actuator / Plant
       ↓
Sensors
```

MPC là lớp dự đoán và tối ưu. PID là lớp phản xạ nhanh. Event Trigger quyết định **có đáng trả chi phí solve MPC ở thời điểm này hay không**. Safety Authority chỉ được thêm nếu số liệu chứng minh guidance-only không chuyển predicted feasibility thành plant safety đủ tốt.

## Thứ tự phát triển bắt buộc

### Gate 1 — Linear baseline — PASS

- PID ổn định trên plant danh định.
- State-space model rõ ràng.
- Periodic MPC tối ưu control sequence.
- Disturbance tạo model prediction error.
- Có quality + compute metrics.

### Gate 2 — Event-triggered predictive guidance — PASS

- Hybrid ổn định.
- Watchdog tồn tại.
- Trigger gồm prediction error, state change, constraint proximity, watchdog.
- Event MPC giảm solve count đáng kể.
- Compute saving được đo cùng tracking quality.

Metric tổng quát:

```text
J_total = w1 * tracking_error
        + w2 * control_effort
        + w3 * solver_time
        + w4 * constraint_violation
        + w5 * plant_safety_violation
```

### Gate 3A — Condensed QP + box-constrained MPC — PASS

- Prediction matrices `Φ`, `Γ`.
- Objective `0.5 UᵀHU + fᵀU`.
- Condensed objective regression-tested với rollout.
- Box QP baseline.
- Optimality + feasibility diagnostics.

### Gate 3B — General input/rate constrained MPC — PASS

- generalized `AU ≤ b`,
- hard input bounds,
- hard `Δu`,
- sparse polyhedral projection,
- explicit `solved / max-iterations / timeout / infeasible / numerical-failure`,
- actuator-safe fallback semantics,
- regression test độc lập cho hard inequalities.

### Gate 3C — Predicted state/output safety + reproducible experiments — ACTIVE

Đã triển khai:

- predicted position inequalities,
- predicted velocity inequalities,
- predicted output inequalities,
- constraint rows được sinh trực tiếp từ `Φ`, `Γ`, free response và `C`,
- safety-envelope preset,
- disturbance-stress preset,
- infeasible-envelope guard,
- model-feasibility metric,
- actual-plant safety metric,
- active state/output constraint metric,
- versioned experiment schema `mpc-pid-experiment/v1`,
- local save/load,
- JSON import/export,
- reproducible solver + safety benchmark,
- CI artifacts chứa smoke + benchmark logs.

PASS khi:

1. nominal periodic constrained MPC giữ predicted envelope khả thi,
2. impossible envelope trả explicit infeasible,
3. fallback vẫn giữ actuator/rate constraints,
4. experiment export → import tái tạo cấu hình tương đương,
5. benchmark chạy tái lập được,
6. predicted feasibility và actual plant safety được báo riêng.

### Decision Gate 3D — Safety Authority

Đây **không phải tính năng mặc định phải thêm**. Quyết định dựa trên benchmark.

Nếu:

```text
predicted QP violation ≈ 0
nhưng
actual hybrid plant safety violation > acceptable threshold
```

thì guidance-only không đủ authority để gọi state/output constraints là hard safety của toàn closed-loop.

Khi đó triển khai theo thứ tự:

1. admissibility monitor cho PID proposal,
2. reference governor hoặc one-step safety filter,
3. MPC first-move / feasible-set based command guard,
4. fallback PID/LQR khi safety solver fail,
5. sau này mới cân nhắc Control Barrier Function cho nonlinear plant.

Mục tiêu không phải để MPC thay PID, mà để PID vẫn phản xạ nhanh nhưng **không được phép phát lệnh khiến predicted safe set mất khả thi**.

Nếu benchmark chứng minh hybrid guidance-only đủ an toàn trong phạm vi nghiên cứu hiện tại, Gate 3D có thể tạm DEFER và chuyển sang State Estimation.

### Gate 4 — State estimation

PASS khi:
- Measurement noise được mô phỏng.
- Observer/Kalman Filter cải thiện estimate.
- Controller không còn giả định đo trực tiếp mọi state.
- Prediction dùng estimated state, không dùng ground-truth simulation state.

Sau đó mới đi EKF/UKF.

### Gate 5 — Nonlinear plants

Thứ tự:
1. UGV bicycle model.
2. UAV planar/attitude model.
3. USV planar model.

Không nhảy vào full 6-DOF UAV trước planar models PASS.

### Gate 6 — NMPC

PASS khi:
- nonlinear model rõ,
- warm start,
- timing budget,
- fallback controller,
- extreme constraint scenarios.

### Gate 7 — Adaptive / Learning MPC

Chỉ sau classical baseline PASS.

AI/ML dùng trước cho:
- residual dynamics,
- disturbance estimation,
- online parameter identification,
- adaptive trigger threshold.

Không cho neural network thay toàn bộ safety-critical control loop ở giai đoạn đầu.

### Gate 8 — Hardware

PASS khi:
- fixed timing budget,
- telemetry log,
- HIL/SIL comparison,
- deadline miss counter,
- fallback PID/LQR,
- reproducible experiment preset/config.

## Quy tắc không đi lạc hướng

1. Mỗi tính năng phải trả lời một câu hỏi nghiên cứu.
2. Mỗi controller phải có baseline.
3. Compute optimization phải đo ảnh hưởng lên control quality.
4. Không thêm AI để “trông hiện đại”.
5. Không tuyên bố real-time khi chưa benchmark target hardware.
6. Không ưu tiên UI hơn correctness của core.
7. Không kết luận tổng quát từ một plant.
8. Mọi experiment quan trọng phải tái lập được.
9. Solver tốt khi **objective + feasibility + optimality + timing** cùng đạt.
10. Solver fail phải có semantics rõ.
11. Penalty không được nhầm với hard constraint.
12. Predicted safety không được nhầm với actual closed-loop safety.
13. Nếu hybrid không có authority bảo đảm constraint, UI và tài liệu phải nói rõ điều đó.

## Milestone đang khóa

**V0.3C — Safety Envelope + Reproducible Research**

Sau khi benchmark PASS, quyết định giữa hai đường:

```text
Safety gap đáng kể → V0.3D Safety Governor → Gate 4 Kalman
Safety gap nhỏ/chấp nhận được → Gate 4 Kalman trực tiếp
```

External OSQP/WASM backend không được thêm chỉ vì “chuẩn công nghiệp”. Chỉ thêm khi benchmark cho thấy solver hiện tại là bottleneck, cần constraint scale lớn hơn hoặc cần đối chiếu kết quả với backend độc lập.
