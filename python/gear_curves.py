import numpy as np

# Gear ratio presets from curves.h
GEAR_PRESETS = {
    'automatic': {
        6: {
            'torque_converter_slip': 100,
            'ratios': [10, 37, 29, 22, 17, 13, 10],
            'overdrive_ratios': [10, 33, 25, 19, 14, 11, 8]
        },
        4: {
            'torque_converter_slip': 100,
            'ratios': [10, 43, 23, 14, 10],
            'overdrive_ratios': [10, 30, 16, 10, 7]
        },
        3: {
            'torque_converter_slip': 100,
            'ratios': [10, 25, 15, 10]
        }
    },
    'virtual_3_speed': [10, 23, 14, 10, 8],
    'virtual_16_speed': [10, 77, 64, 53, 44, 37, 31, 26, 21, 18, 15, 12, 10, 8, 7, 6, 5]
}

# Throttle curves from curves.h
THROTTLE_CURVES = {
    'linear_high_slip': np.array([
        [0, 0], [83, 200], [166, 260], [250, 320],
        [333, 380], [416, 440], [500, 500], [600, 500],
        [700, 500], [800, 500], [900, 500], [1000, 500]
    ]),
    'linear_low_slip': np.array([
        [0, 0], [83, 120], [166, 196], [250, 272],
        [333, 348], [416, 424], [500, 500], [600, 500],
        [700, 500], [800, 500], [900, 500], [1000, 500]
    ])
}

def re_map(curve: np.ndarray, input_val: float) -> float:
    """Python implementation of C++ reMap function with numpy interpolation"""
    x = curve[:, 0]
    y = curve[:, 1]
    return np.interp(input_val, x, y)