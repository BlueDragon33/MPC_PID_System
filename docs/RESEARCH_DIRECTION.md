# Research Direction — MPC + Event Trigger + PID

## North star

Mục tiêu của dự án không phải chứng minh MPC luôn tốt hơn PID. Mục tiêu là xác định **khi nào prediction đáng giá hơn chi phí tính toán**, đồng thời xây một kiến trúc điều khiển có thể chuyển dần từ mô phỏng sang UAV/UGV/USV và phần cứng thật.

## Kiến trúc nghiên cứu ưu tiên

```text
Mission / Planner
       ↓
Sensors
       ↓
State / disturbance estimator
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
```

MPC là lớp dự đoán và tối ưu. PID là lớp phản xạ nhanh. Event Trigger quyết định **có đáng trả chi phí solve MPC ở thời điểm này hay không**. Safety Governor giữ quyền chấp hành cuối cùng trong miền actuator/plan/safety đã kiểm chứng.

## Research gates

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
- explicit solver status,
- actuator-safe fallback semantics,
- regression test độc lập cho hard inequalities.

### Gate 3C — Predicted state/output safety + reproducible experiments — PASS

Đã khóa bằng regression + CI:

- predicted position / velocity / output inequalities,
- safety-envelope và disturbance-stress presets,
- infeasible-envelope guard,
- model-feasibility và actual-plant safety metrics,
- experiment schema `mpc-pid-experiment/v1`,
- local save/load + JSON import/export,
- reproducible benchmark + CI artifacts.

### Gate 3D — Safety Authority / Governor — PASS

Benchmark chứng minh guidance-only không đủ authority:

```text
Periodic constrained MPC plant violation: 0
Hybrid guidance-only violation:           > 0
Hybrid + Safety Governor violation:        0
```

Kiến trúc hiện tại:

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

Governor không chạy thêm full MPC. Nó kiểm tra PID first move rồi dùng phần MPC plan còn lại làm continuation trong short horizon.

Benchmark đại diện:

```text
Safety-envelope intervention rate: ~2.5%
Actuator/plan conditioning rate:   ~97.5%
Hybrid + Safety plant violation:    0
```

Safety intervention và physical/plan conditioning được đo riêng, tránh gọi nhầm mọi command clamp là safety intervention.

### Gate 4 — Linear state estimation + covariance-aware safety — PASS

Gate này đã được khóa bằng CI run #100.

Đã triển khai:

- seeded Gaussian measurement noise,
- two-state linear Kalman Filter,
- Joseph-form covariance update,
- innovation / Kalman gain / covariance diagnostics,
- controller-state routing: Event Trigger, MPC, PID và Governor dùng `x_hat`,
- ground truth chỉ dùng cho plant simulation + audit,
- deterministic closed-loop regression,
- covariance-aware constraint tightening,
- `kσ` sensitivity benchmark,
- full estimated-state safety regression.

Kết quả đại diện với preset noisy-estimation, `kσ = 1.5`:

```text
measurement RMSE: 0.0986
x-hat RMSE:       0.0546
v-hat RMSE:       0.2161
MPC convergence:  100%
fallback:          0
actual violation:  0
estimated/truth IAE ratio: 0.928
```

Covariance tightening dùng:

```text
Δx = kσ sqrt(Pxx)
Δv = kσ sqrt(Pvv)
Δy = kσ sqrt(C P Cᵀ)
```

MPC/Governor dùng envelope đã co. Actual plant safety vẫn được chấm theo envelope vật lý gốc để không tự che estimation error.

Sensitivity cho thấy tăng sigma không đơn điệu tốt hơn: 2σ trở lên bắt đầu gây solver fallback trong benchmark hiện tại, 3σ còn có thể làm tracking/safety xấu đi do quá bảo thủ. Vì vậy 1.5σ hiện là preset nghiên cứu, không phải hằng số phổ quát.

### Gate 4B — Model mismatch + disturbance-state estimation — ACTIVE

Câu hỏi nghiên cứu kế tiếp:

> Khi plant thật lệch mô hình hoặc chịu disturbance không đo trực tiếp, chỉ Kalman 2-state có đủ cho Event MPC + PID + Governor không?

Thứ tự bắt buộc:

1. tạo plant-truth parameters tách khỏi controller model,
2. benchmark model mismatch khi controller không biết mismatch,
3. thêm augmented disturbance state `d_hat`,
4. so sánh 2-state KF với augmented disturbance observer/KF,
5. cho prediction dùng disturbance estimate chỉ sau khi observer chứng minh có lợi,
6. đo IAE, RMSE, solve density, fallback, safety và disturbance-estimation lag,
7. giữ deterministic seed + reproducible experiment.

Không dùng EKF/UKF ở đây vì plant vẫn tuyến tính. Chỉ chuyển EKF/UKF khi Gate 5 nonlinear plant bắt đầu.

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

Không cho neural network thay toàn bộ safety-critical loop ở giai đoạn đầu.

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
9. Solver tốt khi objective + feasibility + optimality + timing cùng đạt.
10. Solver fail phải có semantics rõ.
11. Penalty không được nhầm với hard constraint.
12. Predicted safety không được nhầm với actual closed-loop safety.
13. Safety intervention không được nhầm với actuator/rate conditioning.
14. Estimator quality phải đo bằng error statistics, không chỉ nhìn curve đẹp.
15. Constraint tightening không được dùng để che actual plant violation.
16. Model mismatch phải được benchmark trước khi thêm adaptive/learning compensation.

## Milestone hiện tại

**Gate 4B — Model mismatch + disturbance-state estimation**

```text
nominal model PASS
      ↓
separate truth model
      ↓
controlled model mismatch benchmark
      ↓
augmented disturbance state
      ↓
d_hat estimation regression
      ↓
optional prediction compensation
      ↓
safety + compute + tracking comparison
```

External OSQP/WASM backend vẫn chỉ được thêm khi benchmark cho thấy solver hiện tại là bottleneck hoặc cần backend độc lập để đối chiếu numerical correctness ở constraint scale lớn hơn.
