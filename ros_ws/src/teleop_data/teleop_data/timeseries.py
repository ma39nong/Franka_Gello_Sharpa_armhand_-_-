import numpy as np


class TimeSeries:
    def __init__(self, time, values):
        self.time = np.asarray(time, dtype=np.float64)
        self.values = np.asarray(values, dtype=np.float64)
        if self.time.ndim != 1 or self.values.shape[0] != self.time.size:
            raise ValueError("Time series dimensions do not match.")
        if self.time.size == 0 or not np.all(np.diff(self.time) >= 0):
            raise ValueError("Time series must be non-empty and ordered.")
        if not np.all(np.isfinite(self.time)) or not np.all(np.isfinite(self.values)):
            raise ValueError("Time series contains non-finite values.")

    @classmethod
    def from_samples(cls, samples):
        if not samples:
            raise ValueError("Time series has no samples.")
        time, values = zip(*samples)
        return cls(time, values)

    def linear(self, query):
        query = np.asarray(query, dtype=np.float64)
        columns = self.values.reshape(self.values.shape[0], -1)
        result = np.column_stack(
            [np.interp(query, self.time, columns[:, index]) for index in range(columns.shape[1])]
        )
        return result.reshape((query.size,) + self.values.shape[1:])

    def hold(self, query):
        query = np.asarray(query, dtype=np.float64)
        indices = np.searchsorted(self.time, query, side="right") - 1
        indices = np.clip(indices, 0, self.time.size - 1)
        return self.values[indices]


def fixed_rate_times(start, end, fps):
    if fps <= 0 or end <= start:
        raise ValueError("Episode interval and frame rate must be positive.")
    count = int(np.floor((end - start) * fps)) + 1
    if count < 2:
        raise ValueError("Episode is too short for the requested frame rate.")
    return start + np.arange(count, dtype=np.float64) / fps
