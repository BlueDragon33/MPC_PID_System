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

### Gate 4B — Model mismatch + disturbance-state estimation — PASS

Gate 4B đã được khóa bằng regression tổng hợp `mpc-pid-gate4b-closeout/v1` và workflow riêng.

Đã chứng minh:

1. truth-plant parameters tách khỏi controller model,
2. model mismatch được benchmark khi controller không biết mismatch,
3. augmented disturbance state `d_hat` chạy trong closed loop,
4. 2-state KF và augmented `x-v-d` observer được so sánh trên cùng scenario,
5. same-seed trace tái lập chính xác measurement / estimate / command / trigger,
6. Event Monitor chỉ dùng `d_hat` sau khi observer regression PASS,
7. affine disturbance compensation đã đi vào condensed QP dưới dạng free-response offset, không bị giả thành control input,
8. actual plant safety, solver feasibility và fallback semantics vẫn được audit riêng.

Closeout scenario đại diện:

```text
2-state mismatch:
  IAE:          1.6244
  MPC solves:   28
  x-hat RMSE:   0.0383
  v-hat RMSE:   0.1674

x-v-d observer + d-hat Event Monitor:
  IAE:          1.6278
  MPC solves:   29
  x-hat RMSE:   0.0368
  v-hat RMSE:   0.1606
  d-hat RMSE:   0.4673
  convergence:  100%
  fallback:     0
  plant safety: 0
```

Standalone disturbance-estimation regression trên pulse đã biết:

```text
d-hat RMSE:        0.1209
active mean d-hat: 1.1642   (truth 1.15)
quiet mean |d-hat|:0.0844
x-hat RMSE:        0.0148
v-hat RMSE:        0.0901
```

Disturbance-aware Event Monitor:

```text
prediction-error triggers: 3 → 1
MPC solves:                30 → 29
average prediction error:  0.01605 → 0.01598
IAE ratio ON/OFF:          1.0004
fallback:                  0
plant violation:           0
```

Affine disturbance compensation trong MPC horizon cũng PASS feasibility/safety:

```text
additive-only @ feasible frontier 0.80:
  IAE OFF/ON: 1.4577 / 1.4548

parameter-only:
  IAE OFF/ON: 2.3774 / 2.3760

combined:
  IAE OFF/ON: 1.6278 / 1.6277

all cases:
  convergence: 100%
  fallback:    0
  infeasible:  0
  plant safety violation: 0
```

Kết luận Gate 4B: `d_hat` có giá trị rõ nhất ở **prediction monitor / model residual explanation**; affine MPC horizon compensation hiện an toàn nhưng lợi ích tracking/compute còn rất nhỏ trên linear plant hiện tại. Vì vậy `mismatch-observer` bật disturbance-aware Event Monitor, còn `mpcDisturbanceCompensationEnabled` vẫn **opt-in**. Không overclaim đây là lợi ích phổ quát.

Không dùng EKF/UKF ở Gate 4B vì plant vẫn tuyến tính.

### Gate 5 — Nonlinear plants — ACTIVE

#### Gate 5A — UGV kinematic bicycle + classical baseline — PASS

Đã triển khai nonlinear 4-state bicycle plant:

```text
state = [x, y, yaw, v]
input = [steering, acceleration]

x_dot   = v cos(yaw)
y_dot   = v sin(yaw)
yaw_dot = v/L tan(steering)
v_dot   = acceleration
```

Classical baseline dùng:

- Stanley-style lateral path following,
- PID speed loop,
- hard steering angle,
- hard steering-rate,
- acceleration/speed bounds,
- deterministic S-path scenario,
- explicit safety corridor + quality/compute metrics.

Regression đại diện:

```text
cross-track RMSE:      0.3058 m
heading RMSE:          0.0782 rad
speed RMSE:            0.6415 m/s
max |cross-track|:     0.6500 m
max |heading error|:   0.2467 rad
max steering rate:     0.9000 rad/s
unsafe samples:        0
final speed:           4.0053 m/s
final x:               84.9448 m
runner compute/step:   ~7.1 us
```

Compute/step ở đây chỉ là simulation timing trên CI runner, **không phải tuyên bố hardware real-time**.

#### Gate 5B — Nonlinear state estimation for UGV bicycle — PASS

Đã triển khai:

- deterministic noisy sensor cho `x/y/yaw/v`,
- 4-state EKF dùng nonlinear bicycle prediction + Jacobian,
- sequential Joseph-form covariance update,
- angle innovation được wrap đúng miền `[-π, π]`,
- controller chỉ dùng estimated state,
- ground truth chỉ dùng cho plant transition + audit,
- same-seed deterministic trace,
- different seed làm measurement/estimate/command trace thay đổi,
- RMSE + covariance + safety metrics.

Kết quả đại diện:

```text
measurement RMSE:
  x:   0.1849 m
  y:   0.1762 m
  yaw: 0.03465 rad
  v:   0.1218 m/s

EKF RMSE:
  x:   0.03861 m
  y:   0.03504 m
  yaw: 0.00835 rad
  v:   0.03393 m/s

truth-state baseline:
  cross-track RMSE: 0.30582 m
  heading RMSE:     0.07819 rad
  speed RMSE:       0.64153 m/s

estimated-state loop:
  cross-track RMSE: 0.30780 m
  heading RMSE:     0.07872 rad
  speed RMSE:       0.64031 m/s
  unsafe samples:   0
  avg covariance trace: 0.00720
```

#### Gate 5C — Constrained predictive control comparison on nonlinear UGV — PASS

Đã triển khai một **constrained predictive governor** trên nonlinear bicycle rollout. Đây chưa phải NMPC của Gate 6 vì không tối ưu cả control sequence và không có nonlinear-program solver/warm start.

Thiết kế:

- classical Stanley + PID vẫn tạo proposal,
- predictive governor tạo tập candidate command hữu hạn quanh proposal,
- mỗi candidate được rollout bằng nonlinear bicycle model,
- loại candidate vi phạm cross-track / heading / speed envelope,
- hard steering, steering-rate, acceleration và speed bounds vẫn được giữ,
- predicted feasibility và actual plant safety được audit riêng,
- comparison dùng cùng EKF seed/config.

A/B đại diện:

```text
classical estimated-state:
  cross-track RMSE: 0.30780 m
  heading RMSE:     0.07872 rad
  speed RMSE:       0.64031 m/s
  control effort:   1.32682
  unsafe samples:   0
  compute/step:     68.64 us

predictive governor:
  cross-track RMSE: 0.12869 m
  heading RMSE:     0.05751 rad
  speed RMSE:       0.63624 m/s
  control effort:   1.18018
  unsafe samples:   0
  compute/step:     81.98 us
  avg governor solve: 0.03193 ms
  max governor solve: 0.53627 ms
  predicted feasibility: 100%
  fallback:          0
```

Trade-off:

```text
cross-track ratio predictive/classical: 0.418
heading ratio:                           0.731
speed ratio:                             0.994
compute ratio:                           1.194
```

Kết luận: prediction đáng giá trong scenario UGV hiện tại vì tracking cải thiện lớn hơn mức tăng compute, nhưng đây vẫn chỉ là một plant/scenario; chưa được phép tổng quát hóa thành kết luận NMPC.

#### Gate 5D — UAV planar/attitude nonlinear classical baseline — PASS

Đã triển khai nonlinear 6-state planar UAV:

```text
state = [x, z, theta, vx, vz, q]
input = [thrust, torque]

x_dot     = vx
z_dot     = vz
theta_dot = q
vx_dot    = -(T/m) sin(theta) - d_x vx
vz_dot    =  (T/m) cos(theta) - g - d_z vz
q_dot     = tau/I - d_q q
```

Classical baseline dùng cascaded control:

- outer loop position/velocity tạo desired horizontal/vertical acceleration,
- desired pitch được suy ra từ thrust vector,
- thrust magnitude bù gravity + tracking demand,
- attitude PD inner loop điều khiển torque,
- hard thrust/torque bounds,
- deterministic forward-flight + altitude-wave scenario,
- actual safety audit tách khỏi tracking metrics.

Regression đại diện:

```text
x RMSE:                 0.2158 m
z RMSE:                 0.0725 m
vx RMSE:                0.1757 m/s
vz RMSE:                0.0611 m/s
attitude tracking RMSE: 0.0184 rad
max |x error|:          0.5790 m
max |z error|:          0.3535 m
max |tilt|:             0.1588 rad
unsafe samples:         0
final vx:               1.1000 m/s
final altitude:         2.0122 m
final x:                19.6643 m
runner compute/step:    ~10.03 us
```

Compute/step chỉ là simulation timing trên CI runner, không phải hardware real-time claim.

#### Gate 5E — UAV planar nonlinear state estimation — PASS

Đã triển khai deterministic noisy measurements và EKF 6 trạng thái cho planar UAV:

- state estimate: `[x, z, theta, vx, vz, q]`,
- nonlinear prediction dùng đúng plant planar Gate 5D,
- analytic Jacobian cho coupling thrust/attitude/velocity,
- sequential Joseph-form covariance update,
- attitude innovation wrap trong `[-π, π]`,
- controller dùng estimated state khi estimation bật,
- ground truth chỉ dùng cho plant propagation và audit,
- same-seed deterministic trace + different-seed sensitivity,
- RMSE/covariance/closed-loop safety regression.

Regression đại diện trên Gate 5E CI:

```text
measurement RMSE:
  x:     0.08119 m
  z:     0.05865 m
  theta: 0.01488 rad
  vx:    0.07995 m/s
  vz:    0.07009 m/s
  q:     0.03922 rad/s

EKF RMSE:
  x:     0.01318 m
  z:     0.00991 m
  theta: 0.00305 rad
  vx:    0.02637 m/s
  vz:    0.02368 m/s
  q:     0.01920 rad/s

estimated-state closed loop:
  x RMSE:                 0.22000 m
  z RMSE:                 0.07212 m
  vx RMSE:                0.17554 m/s
  vz RMSE:                0.06235 m/s
  attitude tracking RMSE: 0.01937 rad
  average covariance:     0.003410
  unsafe samples:         0
```

Gate 5E workflow, full smoke, full benchmark, web build và release-readiness candidate đều PASS trên implementation head.

#### Gate 5F — UAV planar constrained predictive comparison — NEXT

Bước kế tiếp là thêm predictive layer **sau** estimator đã PASS, vẫn giữ classical estimated-state controller làm baseline bắt buộc:

1. nonlinear finite-horizon prediction trên planar UAV,
2. hard thrust/torque và safety-envelope feasibility,
3. predicted feasibility tách khỏi actual plant safety,
4. cùng estimator seed/config để A/B công bằng,
5. đo tracking quality, control effort và compute cost,
6. không gọi là NMPC nếu chưa tối ưu full nonlinear control sequence bằng nonlinear-program solver.

Chỉ sau UAV planar estimation + predictive pipeline mới chuyển USV planar.

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

**Gate 5 — Nonlinear plants — ACTIVE**

```text
Gate 4B linear mismatch/disturbance PASS
      ↓
UGV bicycle nonlinear model        PASS
      ↓
classical path/speed baseline      PASS
      ↓
UGV nonlinear state estimation    PASS
      ↓
constrained predictive comparison PASS
      ↓
UAV planar / attitude baseline     PASS
      ↓
UAV nonlinear state estimation    PASS
      ↓
UAV constrained predictive layer  NEXT
      ↓
USV planar model
```

Không nhảy thẳng vào full 6-DOF UAV hoặc NMPC trước khi UGV bicycle model có baseline, reproducible scenario, safety audit và compute metrics.

External OSQP/WASM backend vẫn chỉ được thêm khi benchmark cho thấy solver hiện tại là bottleneck hoặc cần backend độc lập để đối chiếu numerical correctness ở constraint scale lớn hơn.
