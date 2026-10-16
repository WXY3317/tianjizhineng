# SG100 Position Transform Calculation

## 1. Scope

This document explains the exact calculation path used by the SDK for the `position` field when controlling the hand in realtime mode.

It covers:

- what happens when position transform is enabled
- what happens when position transform is disabled
- the per-joint mapping coefficients and limits
- the formulas used for send and receive paths
- where the logic lives in code

Relevant implementation is in:

- `src/sg100_device.cpp`
- `include/sg100_device.h`

## 2. Control Switch

The SDK exposes a device-level switch:

```cpp
hand->SetPositionTransformEnabled(true);   // transform on
hand->SetPositionTransformEnabled(false);  // transform off
```

Meaning:

- `true`: caller-facing position values are treated as user-space angles in degrees, and the SDK performs internal mapping and scaling.
- `false`: caller-facing position values are treated as raw protocol `float position` values, and the SDK sends/returns them directly.

Query API:

```cpp
bool enabled = hand->IsPositionTransformEnabled();
```

## 3. Affected Interfaces

When the switch is on or off, the following interfaces are affected:

- `SetHandControl(...)`
- `SetJointPositions(...)`
- `SetJointPosition(...)`
- realtime thread resend path from `cached_control_`
- `SetJointCommand(...)` in passthrough single-joint control
- `GetCachedHandStatus()` returned `position`
- `GetCachedPosition(...)`
- `GetCachedPositions(...)`
- `RegisterHandStatusCallback(...)` observed `position`

The following are not affected by this switch:

- `velocity`
- `cur_q`
- `cur_d`
- PID parameters `p/i/d`
- `output_limit`

## 4. Send Path With Transform On

When transform is enabled, the SDK converts the caller-provided position in two steps.

### 4.1 Step 1: User Position To Actual Hand Angle

For each joint, the user input is first clamped to a joint-specific range, then mapped by:

$$
actual\_deg = a_i \cdot clamp(user\_deg) + b_i
$$

Where:

- `user_deg`: the position value passed by the caller, in degrees
- `actual_deg`: the hand-side calibrated physical angle, in degrees
- `a_i`, `b_i`: joint-specific mapping coefficients

### 4.2 Step 2: Actual Hand Angle To Wire Position

The mapped angle is then converted into the protocol `float position` field in radians:

$$
wire\_position = actual\_deg \cdot \frac{\pi}{180}
$$

Where:

- `wire_position`: the `float position` written into `Joint_Control_t` / `Hand_Control_t`, in radians

### 4.3 Combined Formula

With transform on, the complete send-side formula is:

$$
wire\_position = \big(a_i \cdot clamp(user\_deg) + b_i\big) \cdot \frac{\pi}{180}
$$

## 5. Receive Path With Transform On

When transform is enabled, the SDK converts the received protocol `float position` back into caller-facing user angle.

### 5.1 Step 1: Wire Position To Actual Hand Angle

$$
actual\_deg = wire\_position \cdot \frac{180}{\pi}
$$

### 5.2 Step 2: Actual Hand Angle To User Angle

$$
user\_deg = \frac{actual\_deg - b_i}{a_i}
$$

### 5.3 Combined Formula

With transform on, the complete receive-side formula is:

$$
user\_deg = \frac{\Big(wire\_position \cdot \frac{180}{\pi}\Big) - b_i}{a_i}
$$

## 6. Behavior With Transform Off

When transform is disabled, the SDK does not apply any mapping or scaling to the `position` field.

Send path:

$$
wire\_position = input\_position
$$

Receive path:

$$
output\_position = wire\_position
$$

That means:

- the value passed to `SetJointPosition(...)` is written directly to the protocol `position`
- the value stored in status cache is the raw protocol `position`
- callbacks observe the raw protocol `position`

## 7. Per-Joint Mapping Table

The SDK uses the following joint-specific limits and coefficients.

| Joint | User Clamp Range | `a_i` | `b_i` | Actual Angle Formula |
|------|-------------------|------:|------:|----------------------|
| J1 | `[-30, 90]` | `1` | `-90` | `actual = user - 90` |
| J2 | `[-10, 100]` | `-1` | `-10` | `actual = -user - 10` |
| J3 | `[-16, 110]` | `-1` | `-16` | `actual = -user - 16` |
| J4 | `[-90, 10]` | `1` | `90` | `actual = user + 90` |
| J5 | `[-10, 100]` | `-1` | `-10` | `actual = -user - 10` |
| J6 | `[-16, 110]` | `-1` | `-16` | `actual = -user - 16` |
| J7 | `[-10, 100]` | `-1` | `-10` | `actual = -user - 10` |
| J8 | `[-16, 110]` | `-1` | `-16` | `actual = -user - 16` |
| J9 | `[0, 180]` | `-1` | `0` | `actual = -user` |
| J10 | `[-10, 100]` | `-1` | `-10` | `actual = -user - 10` |
| J11 | `[-16, 110]` | `-1` | `-16` | `actual = -user - 16` |
| J12 | `[-90, 90]` | `1` | `0` | `actual = user` |

## 8. Code Locations

Key code locations in `src/sg100_device.cpp`:

- `mapJointAngle(...)`: user angle to actual hand angle
- `dmapJointAngle(...)`: actual hand angle to user angle
- `DegToRad(...)`: degree to radian conversion helper
- `RadToDeg(...)`: radian to degree conversion helper
- `EncodePositionForWire(...)`: send-side position conversion helper
- `DecodePositionFromWire(...)`: receive-side position conversion helper

Public API declarations are in `include/sg100_device.h`:

- `SetPositionTransformEnabled(bool)`
- `IsPositionTransformEnabled() const`

## 9. Worked Example

Example: J1 with `transform on`, user sets `30 deg`.

J1 parameters:

- clamp range: `[-30, 90]`
- `a = 1`
- `b = -90`
Step 1:

$$
actual\_deg = 1 \cdot 30 - 90 = -60
$$

Step 2:

$$
wire\_position = -60 \cdot \frac{\pi}{180}
$$

Simplified:

$$
wire\_position \approx -1.0472
$$

So if transform is on, `SetJointPosition(1, 30.0f)` does not send `30.0f`.
It sends approximately `-1.0472` in the protocol `position` field.

If transform is off, the same call sends exactly `30.0f`.

## 10. Practical Summary

Use transform on when:

- you want the SDK to expose a user-space degree interface
- you want the SDK to apply its joint-specific mapping rules automatically
- you want cached status and callbacks to return user-space degree values

Use transform off when:

- you need to send raw protocol `position` directly
- you are debugging firmware/protocol behavior
- you already have your own external calibration and mapping pipeline

The most important distinction is simple:

- transform on: API-facing `position` means user angle in degrees
- transform off: API-facing `position` means raw protocol float