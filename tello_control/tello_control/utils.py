# tello_control/tello_control/utils.py
def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp x into the interval [lo, hi]."""
    return max(lo, min(hi, x))
