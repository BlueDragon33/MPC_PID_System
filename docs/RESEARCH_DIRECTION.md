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
Safety Governor / admissibility filter
       ↓
Actuator / Plant
       ↓
Sensors
```

MPC là lớp dự đoán và tối ưu. PID là lớp phản xạ nhanh. Event Trigger quyết định **có đáng trả chi phí solve MPC ở thời điểm này hay không**. Safety Governor giữ quyền chấp hành cuối cùng trong miền actuator/plan/safety đã kiểm chứng.

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

### Gate 3C — Predicted state/output safety + reproducible experiments — PASS

Đã khóa bằng regression + CI:

- predicted position inequalities,
- predicted velocity inequalities,
- predicted output inequalities,
- constraint rows sinh trực tiếp từ `Φ`, `Γ`, free response và `C`,
- safety-envelope preset,
- disturbance-stress preset,
- infeasible-envelope guard,
- model-feasibility metric,
- actual-plant safety metric,
- active state/output constraint metric,
- versioned experiment schema `mpc-pid-experiment/v1`,
- local save/load,
- JSON import/export,
- roundtrip serialization regression,
- reproducible solver + safety benchmark,
- CI artifacts chứa smoke + benchmark logs.

Nominal periodic constrained MPC giữ plant safety violation bằng 0 trong safety-envelope regression. Impossible envelope trả explicit infeasible và fallback vẫn giữ actuator/rate constraints.

### Gate 3D — Safety Authority / Governor — PASS

Benchmark chứng minh guidance-only không đủ authority:

```text
Periodic constrained MPC plant violation: 0
Hybrid guidance-only violation:           > 0
Hybrid + Safety Governor violation:        0
```

Kiến trúc đã triển khai:

```text
MPC safe plan
    ↓
predictive guidance
    ↓
PID proposal
    ↓
actuator + slew + plan-continuation conditioning
    ↓
short-horizon state/output admissibility check
    ↓
safe command
```

Governor không chạy thêm một full MPC. Nó chỉ thay first move của PID và dùng phần đuôi MPC plan làm continuation để kiểm tra short horizon.

Các điểm đã khóa:

- dynamic PID limits để anti-windup biết miền admissible,
- MPC plan shifting giữa các event-trigger solves,
- emergency actuator-safe fallback,
- actual plant safety regression,
- safety-envelope intervention và actuator/plan conditioning được đo tách biệt,
- disturbance-stress benchmark thật sự chứa disturbance trong cửa sổ benchmark.

Benchmark đại diện sau khi tách semantics:

```text
Safety-envelope intervention rate: ~2.5%
Actuator/plan conditioning rate:   ~97.5%
Hybrid + Safety plant violation:    0
```

Điều này có nghĩa Governor chỉ phải dùng **state/output safety authority** ở một phần nhỏ mẫu; phần conditioning cao chủ yếu đến từ hard `u`, hard `Δu` và khả năng nối tiếp MPC plan, không được gọi nhầm là safety intervention.

Một phát hiện quan trọng khác: khi guidance-only đưa plant ra khỏi safe region, constrained QP có thể mất recursive feasibility và rơi vào fallback nhiều lần. Governor giúp giữ plant trong miền mà các lần MPC solve sau tiếp tục khả thi.

### Gate 4 — State estimation — ACTIVE (estimator core only)

Đã bắt đầu nhưng **chưa nối estimate vào controller**:

- deterministic seeded Gaussian noise source,
- reusable two-state linear Kalman Filter,
- Joseph-form covariance update,
- innovation / innovation-variance / Kalman-gain diagnostics,
- deterministic estimation smoke test.

Gate 4 chỉ PASS khi:

1. measurement noise được mô phỏng trong closed-loop,
2. Kalman giảm estimation RMSE so với measurement thô,
3. covariance hữu hạn, đối xứng và ổn định,
4. Event Trigger dùng estimated state,
5. MPC prediction dùng estimated state,
6. PID/feedback path không còn đọc trực tiếp simulation ground truth,
7. safety audit vẫn so actual plant với estimated-controller behavior để không che model/estimator error.

Sau Gate 4 mới đi EKF/UKF.

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
13. Safety intervention không được nhầm với actuator/rate conditioning.
14. Estimator quality phải đo bằng error statistics, không chỉ nhìn curve đẹp.

## Milestone hiện tại

**Gate 4 — Linear State Estimation**

Thứ tự thực hiện:

```text
seeded measurement noise
        ↓
standalone linear Kalman core
        ↓
estimation RMSE regression
        ↓
closed-loop sensor interface
        ↓
controller state = x_hat
        ↓
MPC + Event Trigger + PID + Governor with estimated state
        ↓
noise / mismatch / disturbance benchmark
```

External OSQP/WASM backend chưa được thêm chỉ vì “chuẩn công nghiệp”. Chỉ thêm khi benchmark cho thấy solver hiện tại là bottleneck cần xử lý trước plant nonlinear, hoặc cần backend độc lập để đối chiếu numerical correctness ở constraint scale lớn hơn.
