import pytest
from numpy import pi
from pytest_mock import mocker

from opensourceleg.encoder import AS5048A_Encoder
from opensourceleg.utilities import twos_compliment


class EncoderStateMock:
    def __init__(self) -> None:
        self._registers: bytearray = bytearray(256)
        self.open()

    def read_i2c_block_data(self, i2cadr, register, len) -> bytes:
        assert 0 <= register <= 256
        assert 0 < len <= 0xFF
        assert 0 <= register + len <= 256
        return bytes(self._registers[register : register + len])

    def write_i2c_block_data(self, i2cadr, register, data) -> None:
        regadr = register
        for byte in data:
            self._registers[regadr] = byte
            regadr += 1

    def open(self):
        self.diag = {AS5048A_Encoder.FLAG_OCF: True}

    def close(self):
        pass

    @property
    def zero_pos(self):
        reg = AS5048A_Encoder.OTP_ZERO_POSITION_HIGH
        return AS5048A_Encoder._get_14bit(self._registers[reg : reg + 2])

    @zero_pos.setter
    def zero_pos(self, value: int):
        reg = AS5048A_Encoder.OTP_ZERO_POSITION_HIGH
        self._registers[reg : reg + 2] = AS5048A_Encoder._set_14bit(value)

    @property
    def diag(self) -> bytes:
        return bytes([self._registers[AS5048A_Encoder.DIAGNOSTICS]])

    @diag.setter
    def diag(self, kwargs: dict[int, bool]):
        reg = AS5048A_Encoder.DIAGNOSTICS
        regvalue = self._registers[reg]
        for (key, item) in kwargs.items():
            if item:
                self._registers[reg] |= key
            else:
                self._registers[reg] &= ~key

    @property
    def angle(self) -> bytes:
        reg = AS5048A_Encoder.ANGLE_HIGH
        return AS5048A_Encoder._get_14bit(self._registers[reg : reg + 2])

    @angle.setter
    def angle(self, value: int):
        reg = AS5048A_Encoder.ANGLE_HIGH
        self._registers[reg : reg + 2] = AS5048A_Encoder._set_14bit(value)


@pytest.fixture()
def encoder_mock() -> EncoderStateMock:
    return EncoderStateMock()


@pytest.fixture()
def patch_encoder(mocker, encoder_mock: EncoderStateMock):
    mocker.patch("smbus2.SMBus.__new__", return_value=encoder_mock)
    mocker.patch("smbus2.SMBus.__init__", return_value=None)


@pytest.fixture
def enc_patched(patch_encoder) -> AS5048A_Encoder:
    obj = AS5048A_Encoder(bus="/dev/null", debug_level=10)
    return obj


def test_mock2(enc_patched: AS5048A_Encoder, encoder_mock: EncoderStateMock):
    enc_patched.start()
    assert enc_patched.encoder_position == 0
    encoder_mock.angle = 2**14 - 1
    enc_patched.update()
    assert enc_patched.encoder_output == 2**14 - 1
    pass


@pytest.fixture
def enc_obj_open(enc_patched: AS5048A_Encoder):
    enc_patched.start()
    return enc_patched


def test_AS5048A_Encoder(enc_obj_open: AS5048A_Encoder):
    assert type(enc_obj_open) == AS5048A_Encoder
    assert enc_obj_open._running == True


def test_encoder_diag(enc_obj_open: AS5048A_Encoder, encoder_mock: EncoderStateMock):
    assert enc_obj_open.diag_OCF == True
    assert enc_obj_open.diag_COF == False
    assert enc_obj_open.diag_compH == False
    assert enc_obj_open.diag_compL == False

    # Test that a false OCF flag raises an error when updating
    with pytest.raises(OSError):
        encoder_mock.diag = {AS5048A_Encoder.FLAG_OCF: False}
        enc_obj_open.update()

    # Test that changing the other registers doesn't change the OCF flag
    with pytest.raises(OSError):
        encoder_mock.diag = {
            AS5048A_Encoder.FLAG_COMP_H: True,
            AS5048A_Encoder.FLAG_COMP_L: True,
            AS5048A_Encoder.FLAG_COF: True,
        }
        enc_obj_open.update()


@pytest.mark.parametrize(
    ("data, exp_pos"),
    [
        (0, 0),
        (1, (2 * pi) / AS5048A_Encoder.ENC_RESOLUTION),
        (
            AS5048A_Encoder.ENC_RESOLUTION - 1,
            -(2 * pi) / AS5048A_Encoder.ENC_RESOLUTION,
        ),
        ((AS5048A_Encoder.ENC_RESOLUTION) * (3 / 4), -pi / 2),
        ((AS5048A_Encoder.ENC_RESOLUTION) * (1 / 4), pi / 2),
    ],
)
def test_update_pos(
    encoder_mock: EncoderStateMock,
    enc_obj_open: AS5048A_Encoder,
    data: int,
    exp_pos: int,
):
    encoder_mock.angle = int(data)
    enc_obj_open.update()

    assert enc_obj_open.encoder_position == pytest.approx(
        exp_pos,
        rel=1 / AS5048A_Encoder.ENC_RESOLUTION,
    )


def test_encoder_velocity_zero(
    encoder_mock: EncoderStateMock, enc_obj_open: AS5048A_Encoder
):
    encoder_mock.angle = 100
    enc_obj_open.update()
    encoder_mock.angle = 100
    enc_obj_open.update()
    assert enc_obj_open.encoder_velocity == 0


def test_encoder_velocity_sign(
    encoder_mock: EncoderStateMock, enc_obj_open: AS5048A_Encoder
):
    # Mock first position
    encoder_mock.angle = 0
    enc_obj_open.update()
    encoder_mock.angle = 1
    enc_obj_open.update()
    pos = enc_obj_open.encoder_velocity
    encoder_mock.angle = 0
    enc_obj_open.update()
    neg = enc_obj_open.encoder_velocity
    assert pos > 0
    assert neg < 0


def test_encoder_velocity_full_scale(
    encoder_mock: EncoderStateMock, enc_obj_open: AS5048A_Encoder
):
    # Mock first position
    encoder_mock.angle = 0
    enc_obj_open.update()
    encoder_mock.angle = AS5048A_Encoder.ENC_RESOLUTION - 1
    enc_obj_open.update()
    enc_obj_open._encdata_old_timestamp = 0
    enc_obj_open._encdata_new_timestamp = 10**9  # 1s in ns

    expected = 2 * pi - (1 / 10**14)
    assert enc_obj_open.encoder_velocity == pytest.approx(
        expected,
        rel=AS5048A_Encoder.ENC_RESOLUTION,
    )


def test_encoder_zero_pos(
    encoder_mock: EncoderStateMock, enc_obj_open: AS5048A_Encoder
):
    assert enc_obj_open.zero_position == 0
    encoder_mock.zero_pos = 100
    enc_obj_open.update()
    assert enc_obj_open.zero_position == 100

    enc_obj_open.zero_position = 200
    assert enc_obj_open.zero_position == 200
    assert encoder_mock.zero_pos == 200


@pytest.mark.parametrize(
    ("a1, a2, expected"),
    [
        (0, 0, 0b1000000),
        (0, 1, 0b1000010),
        (1, 0, 0b1000001),
        (1, 1, 0b1000011),
        (True, True, 0b1000011),
        (100, 100, 0b1000011),
        (100, False, 0b1000001),
    ],
)
def test_calculate_I2C_adress(a1, a2, expected):
    assert AS5048A_Encoder._calculate_I2C_adress(a1, a2) == expected
    assert AS5048A_Encoder(A1_adr_pin=a1, A2_adr_pin=a2).addr == expected


@pytest.mark.parametrize(
    ("byteobject, value"),
    [
        (bytes([0, 0]), 0),
        (bytes([0, 0x3F]), 0x3F),
        (bytes([0xFF, 0xFF]), (2**14) - 1),
        (bytes([0xFF, 0x3F]), (2**14) - 1),
    ],
)
def test_get14Bit(byteobject: bytes, value: int):
    assert value == AS5048A_Encoder._get_14bit(byteobject)
    ##assert byteobject == AS5048A_Encoder._set14Bit(value)
    ##assert AS5048A_Encoder._get14Bit(byteobject) == AS5048A_Encoder._set14Bit(value)


@pytest.mark.parametrize(
    ("byteobject, value, raises"),
    [
        (bytes([0, 0]), 0, None),
        (bytes([0, 0x3F]), 0x3F, None),
        (bytes([0xFF, 0x3F]), (2**14) - 1, None),
        (bytes([0xFF, 0xFF]), (2**14), OverflowError),
    ],
)
def test_set14Bit(byteobject: bytes, value: int, raises):
    if raises:
        with pytest.raises(raises):
            assert byteobject == AS5048A_Encoder._set_14bit(value)
            assert value == AS5048A_Encoder._get_14bit(byteobject)
    else:
        assert byteobject == AS5048A_Encoder._set_14bit(value)
        assert value == AS5048A_Encoder._get_14bit(byteobject)
        assert value == AS5048A_Encoder._get_14bit(AS5048A_Encoder._set_14bit(value))
