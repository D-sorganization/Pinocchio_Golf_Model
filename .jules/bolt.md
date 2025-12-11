## 2025-12-10 - [Recursive Lookup Bottleneck in Exporters]
**Learning:** The `MJCFExporter` used a recursive function that iterated over the entire segment list `self.spec["segments"]` to find children of the current node. This resulted in O(N^2) complexity, causing significant slowdowns for models with many segments (>1000).
**Action:** When working with hierarchical data stored in flat lists, always pre-process the list into an adjacency map (`parent -> children`) to allow O(1) lookups during recursion, reducing overall complexity to O(N).

## 2025-12-10 - [Pinocchio Backend Bias Forces Bug & Optimization]
**Learning:** The previous implementation of `compute_bias_forces` in `PinocchioBackend` relied on `pin.rnea` updating `self.data.nle` as a side effect, which it does NOT do. This meant the function was returning uninitialized data (zeros). Additionally, it was allocating `np.zeros` unnecessarily.
**Action:** Use `pin.nle(model, data, q, v)` directly. It is faster (no allocation, no redundant gravity call) and CORRECT (updates and returns the actual NLE vector). Always verify what side effects C++ bindings actually have.
