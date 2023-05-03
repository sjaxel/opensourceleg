import pytest
import pytest_mock
from pytest_mock import mocker

from opensourceleg.encoder import AS5048A_Encoder
from smbus2 import SMBus
from numpy import pi 

@pytest.fixture
def enc_obj(mock_smbus):
    "Test docstring"
    obj = AS5048A_Encoder(bus='/dev/null')
    return obj

@pytest.fixture
def enc_obj_open(mocker: pytest_mock, enc_obj: AS5048A_Encoder):
    mock_i2c_return_data(mocker, bytes(range(6)))
    enc_obj.open()
    return enc_obj

def mock_i2c_return_data(mocker, data: bytes):
    m = mocker.patch("smbus2.SMBus.read_i2c_block_data")
    m.return_value = data
    return m

def test_AS5048A_Encoder(enc_obj: AS5048A_Encoder, 
                        enc_obj_open: AS5048A_Encoder
    ):
    assert type(enc_obj) == AS5048A_Encoder
    assert enc_obj_open._SMBus.funcs == 0x0eff0009
    assert enc_obj_open._isOpen == True
    

@pytest.fixture()
def mock_smbus(monkeypatch, block_data=None):
    SMBUS_FUNCS = 0x0eff0009 
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
        (bytes([0, 0xFF, 0, 0, 0xFF, 0x3F]), 2*pi),
        (bytes([0, 0xFF, 0, 0, 0x7F, 0x3F]), pi),
        (bytes([0, 0xFF, 0, 0, 0x6F, 0x3F]), pi/2)
    ]
)
def test_update_pos(mocker, enc_obj_open: AS5048A_Encoder, data: bytes, exp_pos: float):
    m = mock_i2c_return_data(mocker, data)
    enc_obj_open.update()
    assert enc_obj_open.encoder_position == pytest.approx(exp_pos, 
                                                            rel=AS5048A_Encoder.ENC_RESOLUTION, 
                                                            )

   
@pytest.mark.parametrize(
    ("test_data"),
    [
        bytes(range(5)),
        b'',
        bytes(range(100))
    ]
)
def test_readRegisters(mocker, enc_obj_open: AS5048A_Encoder, test_data: bytes):
    m = mock_i2c_return_data(mocker, test_data)
    assert enc_obj_open._isOpen == True
    assert enc_obj_open._readRegisters(5, len(bytes(test_data))) == test_data

@pytest.mark.parametrize(
    ("a1, a2, expected"),
    [
        (0, 0, 0b1000000),
        (0, 1, 0b1000010),
        (1, 0, 0b1000001),
        (1, 1, 0b1000011),
        (True, True, 0b1000011),
        (100, 100, 0b1000011),
        (100, False, 0b1000001)
    ]
)
def test_calculateI2CAdress(a1, a2, expected):
    assert AS5048A_Encoder._calculateI2CAdress(a1, a2) == expected
    assert AS5048A_Encoder(A1_adr_pin=a1, A2_adr_pin=a2).addr == expected