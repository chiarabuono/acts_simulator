import numpy as np
TAU_MIN = 5.5
TAU_MAX = 100.0  # PLACEHOLDER - replace with your real winch/cable ceiling
W_MIN = 5.0       # currently unused by the live tension planner (Eq. 2.37
                  # pins ground tension to TAU_MIN) - ready for when the
                  # real Eq. 2.35/2.36 QP is implemented
D_SAFE = 0.4
PAYLOAD_HALF_EXTENTS = np.array([1.0, 1.0, 0.1]) # 2 m × 2 m × 0.2 m platform 