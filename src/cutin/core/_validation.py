"""Small, internal validation helpers for domain records."""

from math import isfinite


def number(name: str, value: float, *, minimum: float | None = None) -> None:
    """Reject booleans and non-finite numbers, including oversized JSON integers."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        finite = isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f"{name} must be a finite number")
    if minimum is not None and value < minimum:
        raise ValueError(f"{name} must be >= {minimum}")


def positive(name: str, value: float) -> None:
    number(name, value, minimum=0)
    if value == 0:
        raise ValueError(f"{name} must be positive")


def integer(name: str, value: int, *, minimum: int = 0) -> None:
    if type(value) is not int or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
