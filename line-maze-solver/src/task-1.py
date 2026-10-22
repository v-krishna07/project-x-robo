# task-1.py
# Task 1: Sensor normalization, weighted centroid line position, and motor clamping

POSITIONS = (-2, -1, 0, 1, 2)

NUM_SENSORS = 5


def get_sensor_limit():
    sensor_min = []
    sensor_max = []
    for i in range(NUM_SENSORS):
        print(f"\nSensor {i + 1}")
        min_val = int(input("  Minimum reading: "))
        max_val = int(input("  Maximum reading: "))
        if min_val > max_val:
            print("  Minimum is larger than maximum, swapping them.")
            min_val, max_val = max_val, min_val
        sensor_min.append(min_val)
        sensor_max.append(max_val)
    return sensor_min, sensor_max


def normalization(sensor_max, sensor_min, sensor_reading):
    span = sensor_max - sensor_min
    if span == 0:
        return 0.0
    value = (sensor_reading - sensor_min) * 100.0 / span
    if value < 0.0:
        return 0.0
    if value > 100.0:
        return 100.0
    return value


def line_position_estimation(sensor_readings):
    total = sum(sensor_readings)
    if total <= 0:
        return None
    weighted = sum(w * s for w, s in zip(POSITIONS, sensor_readings))
    return weighted / total


def limit_motor_commands(left, right, max_magnitude):
    if max_magnitude < 0:
        raise ValueError("max_magnitude must be non-negative")

    def clamp(v):
        if v > max_magnitude:
            return max_magnitude
        elif v < -max_magnitude:
            return -max_magnitude
        else:
            return v

    return clamp(left), clamp(right)


if __name__ == "__main__":
    print("--- Normalization Examples ---")
    # halfway between 200 and 1000 -> 50.0
    print("norm(1000, 200, 600)  =", normalization(1000, 200, 600))
    # above 1000 -> clamped to 100.0
    print("norm(1000, 200, 1500) =", normalization(1000, 200, 1500))

    print("\n--- Line Position Examples ---")
    # center sensor only -> 0.0
    print("center [0, 0, 100, 0, 0]   ->", line_position_estimation([0, 0, 100, 0, 0]))
    # slightly left -> -0.333
    print("left   [0, 50, 100, 0, 0]  ->", line_position_estimation([0, 50, 100, 0, 0]))
    # far left -> -1.5
    print("far L  [100, 100, 0, 0, 0] ->", line_position_estimation([100, 100, 0, 0, 0]))
    # all zero (lost line) -> None
    print("lost   [0, 0, 0, 0, 0]     ->", line_position_estimation([0, 0, 0, 0, 0]))

    print("\n--- Motor Command Clamping Examples ---")
    # right exceeds -255 -> clamped to (200, -255)
    print("clamp(200, -300, 255) =", limit_motor_commands(200, -300, 255))
    # both inside range -> (-50, 30)
    print("clamp(-50, 30, 255)   =", limit_motor_commands(-50, 30, 255))
    # right exceeds +255 -> (0, 255)
