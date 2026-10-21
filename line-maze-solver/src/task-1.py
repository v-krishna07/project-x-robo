POSITIONS = (-2, -1, 0, 1, 2)

def get_sensor_limit():
    """Taking maximum and minimum values of sensors"""
    sensor_min = []
    sensor_max = []
    for i in range(num_sensors):
        print(f"\nSensor {i + 1}")
        min = int(input("  Minimum reading: "))
        max = int(input("  Maximum reading: "))
        if min > max:
            print("  Minimum is larger than maximum, swapping them.")
            min, max = max, min
        sensor_min.append(min)
        sensor_max.append(max)
    return sensor_min, sensor_max


def normalization(sensor_max, sensor_min, sensor_reading):
    """Normalizing the reading to (0,100): 0 = background and 100 = line, """
    span = sensor_max - sensor_min
    if span == 0: # edge case
        return 0.0
    value = (sensor_reading - sensor_min) * 100 / span
    return value if 0<=value <=100 else (0 if value<0 else 100)

def line_position_estimation(sensor_readings):
    """Weighted centroid of 5 readings and 0 case handling"""
    total = sum(sensor_readings)
    if total == 0: # end point or starting point or without line
        return None
    weighted = sum(w * s for w, s in zip(POSITIONS, sensor_readings))
    return weighted / total


def limit_motor_commands(left, right, max_magnitude):
    """clamping in 0,max"""
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
    # norm
    print(normalization(1000, 200, 600))   
    print(normalization(1000, 200, 1500))
    # line_position_estimation
    print(line_position_estimation([0, 0, 100, 0, 0]))    
    print(line_position_estimation([0, 50, 100, 0, 0])) 
    print(line_position_estimation([100, 100, 0, 0, 0]))
    print(line_position_estimation([0, 0, 0, 0, 0])) 
    # limit_motor_commands
    print(limit_motor_commands(200, -300, 255))
    print(limit_motor_commands(-50, 30, 255))
    print(limit_motor_commands(0, 300, 255))