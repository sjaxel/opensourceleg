import pytest

from opensourceleg.utilities import twos_compliment


@pytest.mark.parametrize(
    ("value, bit_length, expected"),
    [
        (0, 8, 0),
        (0, 1, 0),
        (1, 8, 1),
        (0x7F, 8, 0x7F),
        (0x80, 8, -0x80),
        (0xFF, 8, -1),
        (0xFFFF, 16, -1),
        (0x8000, 16, -0x8000),
    ],
)
def test_twos_compliment(value, bit_length, expected):
    assert twos_compliment(value, bit_length) == expected
