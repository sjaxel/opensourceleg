import pytest
import pytest_mock
from numpy import pi
from pytest_mock import mocker
from smbus2 import SMBus

from opensourceleg.encoder import AS5048A_Encoder


@pytest.fixture
def enc_obj(mock_smbus):
    "Test docstring"
    obj = AS5048A_Encoder(bus="/dev/null", debug_level=10)
    return obj


@pytest.fixture
def enc_obj_open(mocker, enc_obj: AS5048A_Encoder):
    mock_i2c_return_data(mocker, bytes(range(6)))
    enc_obj.open()
    return enc_obj


def mock_i2c_return_data(mocker, data: bytes):
    m = mocker.patch("smbus2.SMBus.read_i2c_block_data")
    m.return_value = data
    return


def mock_encoder_data_reg_state(
    mocker,
    agc: int = 0,
    diag: int = AS5048A_Encoder.FLAG_OCF,
    mag: int = 0,
    angle: int = 0,
):
    regs = bytearray(6)
    regs[0] = agc
    regs[1] = diag
    regs[2] = mag & 0xFF
    regs[3] = (mag >> 8) & 0x3F
    regs[4] = angle & 0xFF
    regs[5] = (angle >> 8) & 0x3F

    data = bytes(regs)
    mock_i2c_return_data(mocker, data)


def test_AS5048A_Encoder(enc_obj: AS5048A_Encoder, enc_obj_open: AS5048A_Encoder):
    assert type(enc_obj) == AS5048A_Encoder
    assert enc_obj_open._SMBus.funcs == 0x0EFF0009
    assert enc_obj_open._isOpen == True


@pytest.fixture()
def mock_smbus(monkeypatch, block_data=None):
    SMBUS_FUNCS = 0x0EFF0009
    """
    Mock function for the SMBus I2C interface

    From RPI4 with neurobionics Rasbian build
    SMBUS_FUNCS =
        SMBUS_EMUL|SMBUS_I2C_BLOCK|SMBUS_WRITE_I2C_BLOCK
        |SMBUS_READ_I2C_BLOCK|SMBUS_WRITE_BLOCK_DATA
        |SMBUS_PROC_CALL|SMBUS_WORD_DATA|SMBUS_WRITE_WORD_DATA
        |SMBUS_READ_WORD_DATA|SMBUS_BYTE_DATA|SMBUS_WRITE_BYTE_DATA
        |SMBUS_READ_BYTE_DATA|SMBUS_BYTE|SMBUS_WRITE_BYTE
        |SMBUS_READ_BYTE|SMBUS_QUICK|SMBUS_PEC|I2C
    
    """

    def mock_open(self, bus):
        self.funcs = SMBUS_FUNCS

    monkeypatch.setattr(SMBus, "open", mock_open)


@pytest.mark.parametrize(
    ("data, exp_pos"),
    [
        (bytes([0, 0xFF, 0, 0, 0, 0]), 0),
        (bytes([0, 0xFF, 0, 0, 0xFF, 0x3F]), 2 * pi),
        (bytes([0, 0xFF, 0, 0, 0x7F, 0x3F]), pi),
        (bytes([0, 0xFF, 0, 0, 0x6F, 0x3F]), pi / 2),
    ],
)
def test_update_pos(mocker, enc_obj_open: AS5048A_Encoder, data: bytes, exp_pos: float):
    m = mock_i2c_return_data(mocker, data)
    enc_obj_open.update()
    assert enc_obj_open.encoder_position == pytest.approx(
        exp_pos,
        rel=AS5048A_Encoder.ENC_RESOLUTION,
    )


@pytest.mark.parametrize(("test_data"), [bytes(range(5)), b"", bytes(range(100))])
def test_readRegisters(mocker, enc_obj_open: AS5048A_Encoder, test_data: bytes):
    m = mock_i2c_return_data(mocker, test_data)
    assert enc_obj_open._isOpen == True
    assert enc_obj_open._readRegisters(5, len(bytes(test_data))) == test_data


def test_encoder_velocity_zero(enc_obj_open: AS5048A_Encoder, mocker):
    mock_encoder_data_reg_state(mocker, angle=0)
    enc_obj_open.update()
    enc_obj_open.update()
    assert enc_obj_open.encoder_velocity == 0


def test_encoder_velocity_sign(enc_obj_open: AS5048A_Encoder, mocker):
    # Mock first position
    mock_encoder_data_reg_state(mocker, angle=0)
    enc_obj_open.update()
    mock_encoder_data_reg_state(mocker, angle=1)
    enc_obj_open.update()
    pos = enc_obj_open.encoder_velocity
    mock_encoder_data_reg_state(mocker, angle=0)
    enc_obj_open.update()
    neg = enc_obj_open.encoder_velocity
    assert pos > 0
    assert neg < 0


def test_encoder_velocity_full_scale(enc_obj_open: AS5048A_Encoder, mocker):
    # Mock first position
    mock_encoder_data_reg_state(mocker, angle=0)
    enc_obj_open.update()
    mock_encoder_data_reg_state(mocker, angle=enc_obj_open.max_encoder_counts - 1)
    enc_obj_open.update()
    enc_obj_open._encData_old_timestamp = 0
    enc_obj_open._encData_new_timestamp = 10**9  # 1s in ns

    expected = 2 * pi - (1 / 10**14)
    assert enc_obj_open.encoder_velocity == pytest.approx(
        expected,
        rel=AS5048A_Encoder.ENC_RESOLUTION,
    )


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
def test_calculateI2CAdress(a1, a2, expected):
    assert AS5048A_Encoder._calculateI2CAdress(a1, a2) == expected
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
    assert value == AS5048A_Encoder._get14Bit(byteobject)
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
            assert byteobject == AS5048A_Encoder._set14Bit(value)
            assert value == AS5048A_Encoder._get14Bit(byteobject)
    else:
        assert byteobject == AS5048A_Encoder._set14Bit(value)
        assert value == AS5048A_Encoder._get14Bit(byteobject)
        assert value == AS5048A_Encoder._get14Bit(AS5048A_Encoder._set14Bit(value))
