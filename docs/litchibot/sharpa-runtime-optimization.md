# Sharpa runtime optimization and rollback

This change applies to managed MANUS normal/fake sessions on the next driver
start. Other hand sources and supervised validation keep their existing path.

- The owner's UDP sockets on ports 50011 and 50021 request a 4 MiB receive
  buffer. Linux normally reports 8 MiB after accounting overhead. Existing larger
  buffers are retained; other processes, ports and system sysctls are untouched.
  Check startup `SHARPA_UDP_BUFFER` lines for the actual kernel result.
- The ten tactile channels are polled two at a time. With ten channels and the
  existing 60 Hz setting, the timer runs at 300 Hz and each channel still gets
  60 nonblocking polls per second. All SDK calls remain on their original owner
  thread. This reduces work per callback; it is not a guarantee that an
  individual SDK call cannot block.
- `SHARPA_TIMING` reports calls exceeding 10 ms and state callback intervals
  exceeding 55 ms. It covers state publication, command sending, individual
  tactile reads and MANUS management. Fault-code snapshots are checked on the
  SDK owner every five seconds and logged as `SHARPA_FAULT` when changed.
  Diagnostic logging uses a bounded background worker.

The device permit, stop path, joint limits, state publishing rate, command
freshness settings and recorder continuity thresholds are not changed. The
existing live process must be restarted through its normal launcher to apply
the new code. The source Python files require no ROS rebuild.

## Version switch

From the repository directory:

```bash
python3 ops/run/sharpa_runtime_mode.py status
python3 ops/run/sharpa_runtime_mode.py legacy
# Close and restart the normal teleoperation entry to restore old behavior.
python3 ops/run/sharpa_runtime_mode.py optimized
# Close and restart to enable the optimization again.
```

`legacy` restores the old all-fingers callback and skips UDP tuning and the new
diagnostics. Socket buffers reset with the old process, so the next process has
the original defaults. An explicit `SHARPA_RUNTIME_OPTIMIZATION=0` or `=1` in the
launch environment overrides the file. Missing or invalid settings fail back to
legacy behavior.

## Validation

Tests use mock hardware and local test UDP sockets. They verify socket ownership
and unrelated buffers, no buffer shrinking, exact per-channel poll coverage for
odd and even channel counts, SDK thread ownership, slow diagnostic publication,
and existing MANUS command/limit/stop/recovery paths. Physical dynamic testing is
required to evaluate the previous 132 ms publication gaps; those gaps were not
reproduced by the preceding idle full-stream recording.
