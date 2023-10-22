import numpy as np
from typing import Any

def deadband(value, band_radius):
    return max(value - band_radius, 0) + min(value + band_radius, 0)


def clipped_first_order_filter(input, target, max_rate, tau):
    rate = (target - input) / tau
    return np.clip(rate, -max_rate, max_rate)

def value_or_default(value: Any, default: Any) -> Any:
    return default if value is None else value
