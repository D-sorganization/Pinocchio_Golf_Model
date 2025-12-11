# Palette's UX Journal

## 2025-11-30 - Accessible Slider Controls
**Learning:** In PyQt GUIs, simply adding `label.setBuddy(slider)` enables click-to-focus, significantly improving keyboard accessibility without extra event handling.
**Action:** Always pair `QLabel` with its input widget using `setBuddy` when creating form-like controls.
