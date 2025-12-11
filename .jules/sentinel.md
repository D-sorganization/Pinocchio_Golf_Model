## 2024-05-24 - Sandbox Escape via ast.Attribute in Eval
**Vulnerability:** `ast.Attribute` was allowed in `ExpressionFunction`'s AST whitelist. This allowed attribute access on objects (like literals), potentially leading to sandbox escape via traversal (e.g., `(1).__class__`).
**Learning:** Even with restricted `__builtins__`, allowing attribute access in `eval` contexts significantly increases the attack surface for Python sandbox escapes.
**Prevention:** Strictly whitelist AST nodes. Do not allow `ast.Attribute` unless absolutely necessary and strictly validated. Prefer specific function calls (via `ast.Name` check) over general attribute access.
