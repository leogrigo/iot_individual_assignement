# iot_individual_assignement

## Performance Comparison (Windows considered: No Adaptive → id 3–21, Adaptive → id 7–27)

| Metric                  | No Adaptive Sampling | Adaptive Sampling |
|------------------------|---------------------|-------------------|
| Samples per window     | ~72,369             | 457               |
| Execution time (µs)    | ~116,600            | ~1,458            |
| Exec time std dev (µs) | ~450                | ~60               |
| Cost per sample (µs)   | ~1.6                | ~3.2              |
| Window duration (ms)   | ~5000               | ~5002             |

### Computation Saving Ratio

- Execution time reduction: **~80×**
- Samples reduction: **~158×**