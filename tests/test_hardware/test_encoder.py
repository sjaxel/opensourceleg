import pytest

from opensourceleg.encoder import AS5048A_Encoder as Encoder

def test_AS5048A_Encoder():
    assert type(Encoder()) == Encoder

@pytest.mark.parametrize(
    ("a1, a2, expected"),
    [
        (0, 0, 0b1000000),
        (0, 1, 0b1000010),
        (1, 0, 0b1000001),
        (1, 1, 0b1000011)
    ]
)
def test_calculateI2CAdress(a1, a2, expected):
    assert Encoder._calculateI2CAdress(a1, a2) == expected
    assert Encoder(A1_adr_pin=a1, A2_adr_pin=a2).addr == expected