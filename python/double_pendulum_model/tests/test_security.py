import pytest
from double_pendulum_model.physics.double_pendulum import ExpressionFunction, DoublePendulumState

def test_expression_no_attributes() -> None:
    """Ensure accessing attributes is forbidden to prevent sandbox escape."""
    with pytest.raises(ValueError, match="Disallowed syntax in expression: Attribute"):
        ExpressionFunction("(1).__class__")


def test_expression_no_method_calls() -> None:
    """Ensure calling methods on objects is forbidden."""
    # This fails because the Call node validation checks if func is Name,
    # and here func is Attribute.
    with pytest.raises(ValueError, match="Only direct function calls are permitted"):
        ExpressionFunction("t.__str__()")

def test_expression_disallowed_function() -> None:
    """Ensure calling disallowed functions is forbidden."""
    with pytest.raises(ValueError, match="Function 'eval' is not permitted"):
        ExpressionFunction("eval('1')")


def test_expression_valid_math() -> None:
    """Ensure valid math expressions still work."""
    expr = ExpressionFunction("sin(t) + cos(theta1) * 0.5")
    state = DoublePendulumState(theta1=0.0, theta2=0.0, omega1=0.0, omega2=0.0)
    result = expr(0.0, state)
    assert isinstance(result, float)


def test_expression_unknown_variable() -> None:
    """Ensure accessing unknown variables is forbidden."""
    with pytest.raises(ValueError, match="Use of unknown variable '__builtins__'"):
        ExpressionFunction("__builtins__")
