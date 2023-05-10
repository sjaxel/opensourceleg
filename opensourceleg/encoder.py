import logging
import time
from abc import ABC, abstractmethod

import numpy as np
from smbus2 import I2cFunc, SMBus

from opensourceleg.hardware import DEFAULT_UNITS, UnitsDefinition


class Encoder(ABC):
    def __init__(
        self,
        encoder_res: int,
        name: str = "Encoder",
        units: UnitsDefinition = DEFAULT_UNITS,
        logger: logging.Logger = None,
        debug_level: int = 0,
    ) -> None:
        self.debug_level = debug_level
        if logger:
            self.logger = logger.getChild(name)
        else:
            self.logger = logging.getLogger(name)
        self.name = name
        self._state = None
        self._units = units if units else DEFAULT_UNITS
        self._isOpen = False
        self.max_encoder_counts = encoder_res  # 14 bit encoder, so 14 bits!

    def __enter__(self) -> None:
        self.open()

    def __exit__(self, type, value, tb) -> None:
        self.close()

    """
    Establish connection with the encoder device and do any initalization needed

    Args:
        None
    """

    def open(self) -> None:
        if not self._isOpen:

            self._open()
            self.logger.info(f"Open encoder communication {self.__class__.__name__}")

    @abstractmethod
    def close(self) -> None:
        if self._isOpen:
            self._close()

    @abstractmethod
    def update(self) -> None:
        pass

    @property
    @abstractmethod
    def encoder_position(self) -> float:
        """
        The unsigned encoder position in rad[0, 2*pi] as offset
        from the programmed zero position

        Args:
            None

        Returns:
            float: Unsigned offset in rad rad[0, 2*pi]
        """
        pass

    @property
    @abstractmethod
    def encoder_output(self) -> float:
        """
        The encoder output in counts as offset from the programmed zero position

        Returns:
            int: Encoder output in counts[0, ENC_RESOLUTION-1]
        """
        pass

    @property
    @abstractmethod
    def encoder_velocity(self) -> float:
        """
        The encoder position in rad as offset from the programmed zero position

        Returns:
            float: Encoder velocity (rad/s)
        """
        pass


class AS5048A_Encoder(Encoder):
    """Class for the AS5048A encoder, implements the Encoder interface

    https://www.mouser.com/datasheet/2/588/AS5048_DS000298_4_00-2324531.pdf


    Args:
        Encoder (_type_): _description_
        bus (str): Path to the i2c bus ex. '/dev/i2c-1'
        A1_adr_pin (int): State of the adress pin A1 on the AS5048A module
        A2_adr_pin (int): State of the adress pin A1 on the AS5048A module
        name (str): _description_
        units (UnitsDefinition): _description_
        debug_level (int): _description_
    Raises:
        KeyError: _description_
        ValueError: _description_
        KeyError: _description_

    Returns:
        _type_: _description_
    """

    ENC_RESOLUTION = 2**14  # 14 bit resolution

    I2C_BASE_ADR_7BIT = (
        0b1000000  # The adress base on the format <base[6:2]> <A1[1]> <A2[0]>
    )

    ## Register adresses I2C
    OTP_ZERO_POSITION_HIGH = 0x16  # bit 13 through 6
    OTP_ZERO_POSITION_LOW = 0x17  # bit 5 through 0 (2 msbs of this aren't used)
    AUTOMATIC_GAIN_CONTROL = 0xFA  # 0 = high mag field, 255 = low mag field, 8 bit
    DIAGNOSTICS = 0xFB  # flags: 3 = comp high, 2 = comp low, 1 = COF, 0 = OCF
    MAGNITUDE_HIGH = 0xFC  # bit 13 through 6
    MAGNITUDE_LOW = 0xFD  # bit 5 through 0 (2 msbs of this aren't used)
    ANGLE_HIGH = 0xFE  # bit 13 through 6
    ANGLE_LOW = 0xFF  # bit 5 through 0 (2 msbs of this aren't used)

    ## Status flags Diagnostics registers
    FLAG_COMP_H = 0x1 << 3
    FLAG_COMP_L = 0x1 << 2
    FLAG_COF = 0x1 << 1
    FLAG_OCF = 0x1 << 0

    """Class for the AS5048A encoder, implements the Encoder interface

    https://www.mouser.com/datasheet/2/588/AS5048_DS000298_4_00-2324531.pdf


    Args:
        Encoder (_type_): _description_
        bus (str): Path to the i2c bus ex. '/dev/i2c-1'
        A1_adr_pin (int): State of the adress pin A1 on the AS5048A module
        A2_adr_pin (int): State of the adress pin A1 on the AS5048A module
        name (str): _description_
        units (UnitsDefinition): _description_
        debug_level (int): _description_

    Raises:
        KeyError: _description_
        ValueError: _description_
        KeyError: _description_

    Returns:
        _type_: _description_
    """

    def __init__(
        self,
        bus: str = "/dev/i2c",
        A1_adr_pin: bool = False,
        A2_adr_pin: bool = False,
        name: str = "AS5048A_Encoder",
        units: UnitsDefinition = DEFAULT_UNITS,
        logger: logging.Logger = None,
        debug_level: int = 0,
    ) -> None:
        super().__init__(
            AS5048A_Encoder.ENC_RESOLUTION, name, units, logger, debug_level
        )
        self.bus = bus
        self.addr = AS5048A_Encoder._calculateI2CAdress(A1_adr_pin, A2_adr_pin)

        self._encData_old = bytes(6)
        self._encData_old_timestamp = 0
        self._encData_new = bytes(6)
        self._encData_new_timestamp = 0

    def open(self) -> None:
        if not self._isOpen:
            self.logger.info(f"Open encoder communication {self.__class__.__name__}")
            self._SMBus = SMBus(self.bus)
            self.logger.debug(
                f"[OPEN] SMBUS func: {self._SMBus.funcs:>08x} \n\r"
                + str(I2cFunc(self._SMBus.funcs))
            )
        self._isOpen = True
        self.update()

    def close(self) -> None:
        if self._isOpen:
            if hasattr(self, "_SMBus"):
                self._SMBus.close()
            self._isOpen = False

    def update(self) -> None:
        if self._isOpen:
            self._readDataRegisters()
            self._checkDiagnostics()
        else:
            raise OSError("Bus needs to be opened before calling update")

    @staticmethod
    def _calculateI2CAdress(a1: bool, a2: bool) -> int:
        # (a1, a2) = adress_pins
        return AS5048A_Encoder.I2C_BASE_ADR_7BIT | ((bool(a2)) << 1) | ((bool(a1)) << 0)

    @staticmethod
    def _get14Bit(bytesToParse: bytes) -> int:
        return int((bytesToParse[0] << 6) | bytesToParse[1])

    @staticmethod
    def _set14Bit(intToParse: int) -> bytes:
        """
        Convert a 14bit integer to bytes <msb[13:6]><lsb[5:0]>

        Args
            intToParse (int): The integer to convert to bytes

        Raises
            OverflowError: If intToParse >= 2^14
        """
        if intToParse >= AS5048A_Encoder.ENC_RESOLUTION:
            raise OverflowError(
                f"Argument intToParse={intToParse} >= 2^14 bit encoder resolution"
            )
        return bytes([(intToParse >> 6), intToParse & 0x3F])

    def _writeRegisters(self, register: int, data: bytes) -> None:
        self._SMBus.write_i2c_block_data(self.addr, register, data)

    def _readRegisters(self, register, len) -> bytes:
        return bytes(self._SMBus.read_i2c_block_data(self.addr, register, len))

    def _readDataRegisters(self) -> None:
        """
        Read data output registers
            [0]
            [1]
            [2] 0xFC MAG H
            [3] 0xFD MAG L
            [4] 0xFE ANG H
            [5] 0xFF ANG L
        """
        self._encData_old = self._encData_new
        self._encData_old_timestamp = self._encData_new_timestamp

        self._encData_new = self._readRegisters(
            AS5048A_Encoder.AUTOMATIC_GAIN_CONTROL, 6
        )
        self._encData_new_timestamp = time.monotonic_ns()

    def _checkDiagnostics(self):
        if not self.diag_OCF:
            raise OSError("Invalid data returned on read, DIAG_OCF != 1")

        if self.diag_COF:
            self.logger.error("CORDIC Overflow, sample invalid")

        if self.diag_compH:
            self.logger.warning("Low magnetic field comp triggered")

        if self.diag_compL:
            self.logger.warning("High magnetic field comp triggered")

    @property
    def encoder_position(self) -> float:
        return (self.encoder_output * 2 * np.pi) / self.max_encoder_counts

    @property
    def encoder_position_signed(self) -> float:
        """
        The unsigned encoder position in rad[-pi, pi] as offset
        from the programmed zero position

        Returns:
            float: Signed offset in rad rad[-pi, pi]
        """
        raise NotImplementedError()

    @property
    def encoder_output(self) -> int:
        return AS5048A_Encoder._get14Bit(self._encData_new[4:6])

    @property
    def encoder_velocity(self) -> float:
        try:
            encAngleDataOld = AS5048A_Encoder._get14Bit(self._encData_old[4:6])
            encAngleDataNew = AS5048A_Encoder._get14Bit(self._encData_new[4:6])
            # Timediff is converted from ns to s (x10^-9)
            timediff = (
                self._encData_new_timestamp - self._encData_old_timestamp
            ) * 10**-9
        except TypeError:
            ## Typeerror indicate we only took one sample and have 0 velocity.
            return float(0)
        else:
            return (
                (encAngleDataNew - encAngleDataOld)
                * (2 * np.pi)
                / self.max_encoder_counts
                / (timediff)
            )

    @property
    def zero_position_OTP(self) -> int:
        """
        Get:
            Reads the content of the Zero position registers of the Encoder
        Set:
            Sets the zero position OTP registers (but does not burn them)

        Args:
            value (int): The zero position encoder offset

        Raises:
            OverflowError: If value >= 2^14

        Returns:
            int: The 14 bit value stored in the Zero offset OTP registers
        """
        registers = self._readRegisters(AS5048A_Encoder.OTP_ZERO_POSITION_HIGH, 2)
        return AS5048A_Encoder._get14Bit(registers)

    @zero_position_OTP.setter
    def zero_position_OTP(self, value: int):
        """

        Args:
            value (int):

        Raises:
            OverflowError: If value >= 2^14
        """
        enc.logger.info(f"[SET] Zero position OTP: {value}")
        payload = AS5048A_Encoder._set14Bit(value)
        self._writeRegisters(AS5048A_Encoder.OTP_ZERO_POSITION_HIGH, payload)

    @property
    def diag_compH(self) -> bool:
        """
        COMP high, indicated a weak magnetic field. It is
        recommended to monitor the magnitude value.

        Returns:
            Status of COMP_H diagnostics flag
        """
        return bool(self._encData_new[1] & AS5048A_Encoder.FLAG_COMP_H)

    @property
    def diag_compL(self) -> bool:
        """
        COMP low, indicates a high magnetic field. It is
        recommended to monitor in addition the magnitude
        value.

        Returns:
            Status of COMP_L diagnostics flag
        """
        return bool(self._encData_new[1] & AS5048A_Encoder.FLAG_COMP_L)

    @property
    def diag_COF(self) -> bool:
        """
        COF (CORDIC Overflow), logic high indicates an out of
        range error in the CORDIC part. When this bit is set, the
        angle and magnitude data is invalid. The absolute output
        maintains the last valid angular value.
        Returns:
            Status of COF diagnostics flag
        """
        return bool(self._encData_new[1] & AS5048A_Encoder.FLAG_COF)

    @property
    def diag_OCF(self) -> bool:
        """
        OCF (Offset Compensation Finished), logic high indicates
        the finished Offset Compensation Algorithm. After power
        up the flag remains always to logic high.

        Returns:
            Status of OCF diagnostics flag
        """
        return bool(self._encData_new[1] & AS5048A_Encoder.FLAG_OCF)

    def __repr__(self) -> str:
        ang = self.encoder_position
        vel = self.encoder_velocity
        str = f"\n\tAngle: {ang:.3f} rad\n\tVelocity: {vel:.3f} rad/s"
        return str


if __name__ == "__main__":

    logging.basicConfig(
        format="[%(asctime)s][%(name)s][%(levelname)s] %(message)s",
        datefmt="%I:%M:%S",
        level=logging.INFO,
    )
    enc = AS5048A_Encoder(bus="/dev/i2c-1", logger=logging.getLogger())
    with enc:
        enc.zero_position_OTP = 0
        enc.update()
        enc.logger.info(f"Zero registers: {enc.zero_position_OTP}")
        enc.logger.info(f"Enc output: {enc.encoder_output}")
        enc.zero_position_OTP = enc.encoder_output
        enc.logger.info(f"Zero registers: {enc.zero_position_OTP}")
        enc.logger.info(f"Enc output: {enc.encoder_output}")
        enc.update()
        enc.logger.info(f"Zero registers: {enc.zero_position_OTP}")
        enc.logger.info(f"Enc output: {enc.encoder_output}")

        while True:
            enc.update()
            enc.logger.info(f"Position: {enc.encoder_position:.3f}")
            time.sleep(2)
