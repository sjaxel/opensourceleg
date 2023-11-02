from typing import Union

import time
from abc import abstractmethod
from dataclasses import InitVar, dataclass, field

import numpy as np
from smbus2 import SMBus

from opensourceleg.device import DeviceManager, Interface, OSLDevice
from opensourceleg.units import DEFAULT_UNITS, UnitsDefinition


class Loadcell(Interface):
    @property
    @abstractmethod
    def fx(self) -> float:
        pass

    @property
    @abstractmethod
    def fy(self) -> float:
        pass

    @property
    @abstractmethod
    def fz(self) -> float:
        pass

    @property
    @abstractmethod
    def mx(self) -> float:
        pass

    @property
    @abstractmethod
    def my(self) -> float:
        pass

    @property
    @abstractmethod
    def mz(self) -> float:
        pass


@dataclass
class LoadcellCalibration:
    """A class to manage the loadcell calibration matrix and zero offset


    Args:
        nChannels (int): number of loadcell channels
        loadcell_matrix (np.ndarray): loadcell calibration matrix (nChannels x nChannels)
        loadcell_zero (np.ndarray): loadcell zero offset (1 x nChannels)
        amp_gain (float): gain of the strain amplifier
        exc (float): excitation voltage of the strain amplifier

    """

    nChannels: int = field(default=6)  #: number of loadcell channels
    loadcell_matrix: np.ndarray = field(
        default=None
    )  #: loadcell calibration matrix (nChannels x nChannels)
    yaw: float = field(default=0)  #: yaw angle in radians
    pitch: float = field(default=0)  #: pitch angle in radians
    roll: float = field(default=0)  #: roll angle in radians

    loadcell_zero: np.ndarray = field(
        default=None
    )  #: loadcell zero offset (1 x nChannels)
    exc: float = field(default=5)  # excitation voltage of the strain guague

    def __post_init__(self) -> None:
        if self.loadcell_matrix is None:
            self.loadcell_matrix = np.identity(self.nChannels)
        else:
            self.loadcell_matrix = np.array(self.loadcell_matrix)
            if self.loadcell_matrix.shape != (self.nChannels, self.nChannels):
                raise ValueError(
                    f"Loadcell matrix for nChannels={self.nChannels} \
                    must be of shape ({self.nChannels}, {self.nChannels})"
                )
        if self.loadcell_zero is not None:
            self.loadcell_zero = np.array(self.loadcell_zero)
            if self.loadcell_zero.shape != (
                1,
                self.nChannels,
            ) and self.loadcell_zero.shape != (self.nChannels,):
                print(self.loadcell_zero.shape)
                raise ValueError(
                    f"Loadcell zero for nChannels={self.nChannels} \
                    must be of shape ({self.nChannels},) or (1, {self.nChannels})"
                )

        self._calculate_rotational_matrix()

    def apply(self, loadcell_output: np.ndarray) -> np.ndarray:
        """Apply the calibration matrix and zero offset to the loadcell output

        This takes the raw loadcell output from an OUT = [Ch0, Ch1, ... ChN] and
        converts it to mV/V by multiplying by 1000*EXC (excitation voltage).

        Then, the loadcell calibration matrix is applied to convert the loadcell
        output to a wrench in N and Nm.

        Finally, any zero offset is subtracted from the wrench.

        Args:
            loadcell_output (np.ndarray): loadcell output (1 x nChannels) in units of V/V

        """

        # Apply the calibration matrix to the loadcell output
        calibrated: np.ndarray = self.loadcell_matrix.dot(loadcell_output.T)

        # Split the force and moment into column vectors
        force: np.ndarray = calibrated[:3]
        moment: np.ndarray = calibrated[3:]

        # Apply the rotational matrix to the force components
        force = self._rotational_matrix @ force

        # Apply the rotational matrix to the moment components
        moment = self._rotational_matrix @ moment

        # Reassemble the force and moment components
        res = np.concatenate((force.T, moment.T))

        # Apply the zero offset
        if self.loadcell_zero is not None:
            res -= self.loadcell_zero

        return res

    def _calculate_rotational_matrix(self) -> np.ndarray:
        """Calculate the rotational matrix for the loadcell calibration

        This function calculates the rotational matrix for the loadcell calibration
        based on the yaw, pitch, and roll angles. The rotational matrix is used to
        adjust the loadcell output to account for the orientation of the loadcell
        relative to the leg.

        """
        if not any([self.yaw, self.pitch, self.roll]):
            self._rotational_matrix = np.identity(3)
            return

        cy = np.cos(self.yaw)
        sy = np.sin(self.yaw)
        cp = np.cos(self.pitch)
        sp = np.sin(self.pitch)
        cr = np.cos(self.roll)
        sr = np.sin(self.roll)

        self._rotational_matrix = np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ]
        )


class StrainAmp(OSLDevice):

    cal: LoadcellCalibration

    def __init__(self, name: str = "StrainAmp", **kwargs) -> None:
        super().__init__(name=name, **kwargs)

    def apply_state(self, state: Loadcell.State) -> None:
        raise NotImplementedError(f"apply_state not implemented for {self.__class__}")


class FlexSEAStrainAmp(StrainAmp, Loadcell):
    """
    A class to directly manage the 6ch strain gauge amplifier over I2C.
    Author: Mitry Anderson
    """

    nCHANNELS: int = 6
    ADC_GAIN: int = 125  # gain of the strain amplifier
    ADC_RESOLUTION: int = 12  # resolution of the strain amplifier
    ADC_FS: int = 2**ADC_RESOLUTION  # full scale of the strain amplifier (4096)
    ADC_VREF = 5  # volts

    AMP_EXC = 5  # volts

    # register numbers for the "ezi2c" interface on the strainamp
    # found in source code here: https://github.com/JFDuval/flexsea-strain/tree/dev
    MEM_R_CH1_H = 8
    MEM_R_CH1_L = 9
    MEM_R_CH2_H = 10
    MEM_R_CH2_L = 11
    MEM_R_CH3_H = 12
    MEM_R_CH3_L = 13
    MEM_R_CH4_H = 14
    MEM_R_CH4_L = 15
    MEM_R_CH5_H = 16
    MEM_R_CH5_L = 17
    MEM_R_CH6_H = 18
    MEM_R_CH6_L = 19

    def __init__(
        self,
        name: str = "FlexSEAStrainAmp",
        bus: str = "/dev/i2c-1",
        I2C_addr: int = 0x66,
        amplifier_input_offset: np.ndarray = None,
        calibration: LoadcellCalibration = None,
        compressed_data: bool = True,
        **kwargs,
    ) -> None:
        super().__init__(name=name, **kwargs)
        self.bus = bus
        self.addr = I2C_addr
        self._adc_input_offset: np.ndarray
        self._i2c_data_compressed: bool = compressed_data

        if calibration is None:
            self._adc_datacal = LoadcellCalibration(
                nChannels=self.nCHANNELS, exc=self.AMP_EXC
            )
        else:
            self._cal = calibration
            self._cal.exc = self.AMP_EXC

        self._adc_data: np.ndarray

        if amplifier_input_offset is None:
            self._adc_input_offset = np.zeros(self.nCHANNELS)
        else:
            self._adc_input_offset = amplifier_input_offset

        self.output_data: np.ndarray = np.zeros(self.nCHANNELS)
        self.failed_reads = 0

    def read_strain_data(self):
        """Read the adc data from the strain amp

        Used for more recent versions of strain amp firmware

        Raises:
            Exception: If the strain amp fails to respond after 5 attempts

        """
        try:
            if self._i2c_data_compressed:
                i2c_data = self._SMBus.read_i2c_block_data(
                    self.addr, self.MEM_R_CH1_H, 12
                )
                if any(i2c_data[9:12]):
                    raise ValueError(f"Read failed with data: {i2c_data}")
                raw_adc_data: np.ndarray = self._unpack_compressed_strain(i2c_data)

            else:
                i2c_data = self._SMBus.read_i2c_block_data(
                    self.addr, self.MEM_R_CH1_H, 12
                )
                raw_adc_data: np.ndarray = self._unpack_uncompressed_strain(i2c_data)
        except (OSError, ValueError) as e:
            self.failed_reads += 1
            # print("\n read failed")
            if self.failed_reads >= 5:
                raise OSError("Load cell unresponsive.")
            self._log.warning(
                f"Failed to read from strain amp with {self.failed_reads} failed attempts"
            )
        else:
            # Convert to signed code and compensate for the input offset
            self._adc_data = (
                raw_adc_data - (self.ADC_FS // 2)
            ) - self._adc_input_offset
            self.failed_reads = 0

            # Process the data to strain
            self._calculate_strain_data()

    def _start(self):
        self._SMBus = SMBus(self.bus)

    def _update(self):
        self.read_strain_data()

    def _stop(self):
        self._SMBus.close()

    def _unpack_compressed_strain(self, data: list[int]) -> np.ndarray:
        """Used for more recent versions of strainamp firmware"""
        # ch1 = (data[0] << 4) | ( (data[1] >> 4) & 0x0F)
        # ch2 = ( (data[1] << 8) & 0x0F00) | data[2]
        # ch3 = (data[3] << 4) | ( (data[4] >> 4) & 0x0F)
        # ch4 = ( (data[4] << 8) & 0x0F00) | data[5]
        # ch5 = (data[6] << 4) | ( (data[7] >> 4) & 0x0F)
        # ch6 = ( (data[7] << 8) & 0x0F00) | data[8]
        # moved into one line to save 0.02ms -- maybe pointless but eh
        return np.array(
            [
                (data[0] << 4) | ((data[1] >> 4) & 0x0F),
                ((data[1] << 8) & 0x0F00) | data[2],
                (data[3] << 4) | ((data[4] >> 4) & 0x0F),
                ((data[4] << 8) & 0x0F00) | data[5],
                (data[6] << 4) | ((data[7] >> 4) & 0x0F),
                ((data[7] << 8) & 0x0F00) | data[8],
            ]
        )

    def _unpack_uncompressed_strain(self, data: list[int]) -> np.ndarray:
        """Used for an older version of the strain amp firmware (at least pre-2017)"""
        ch1 = (data[0] << 8) | data[1]
        ch2 = (data[2] << 8) | data[3]
        ch3 = (data[4] << 8) | data[5]
        ch4 = (data[6] << 8) | data[7]
        ch5 = (data[8] << 8) | data[9]
        ch6 = (data[10] << 8) | data[11]
        return np.array([ch1, ch2, ch3, ch4, ch5, ch6])

    def _calculate_strain_data(self):
        """Converts strain values between 0 and 4095 to a wrench in N and Nm

        TODO: This function should be split between the channels and moved to the
        property getters to avoid doing unnecessary calculations when strain
        is not accessed.
        """

        mV_per_V = (self._adc_data * 1000) / (
            self.ADC_FS * self.ADC_GAIN
        )  # Code to V_adc
        self.output_data = self.cal.apply(mV_per_V)  # Apply decoupling and zero offset

    @property
    def fx(self) -> float:
        return self.output_data[0]

    @property
    def fy(self) -> float:
        return self.output_data[1]

    @property
    def fz(self) -> float:
        return self.output_data[2]

    @property
    def mx(self) -> float:
        return self.output_data[3]

    @property
    def my(self) -> float:
        return self.output_data[4]

    @property
    def mz(self) -> float:
        return self.output_data[5]

    @property
    def loadcell_data(self) -> [float]:
        return self.output_data

    @property
    def cal(self) -> LoadcellCalibration:
        return self._cal


if __name__ == "__main__":

    decoupling_matrix = [
        (8.50935, -1297.76172, 11.37597, 11.41524, 0.08508, 1279.14832),
        (-26.36361, 734.72449, -14.30058, -1468.19031, 20.55947, 768.81042),
        (-822.97290, -7.42868, -818.24860, -9.61839, -831.10327, 0.99515),
        (16.68220, 0.28728, -0.36670, -0.01215, -17.29246, -0.13635),
        (-9.87459, -0.81627, 19.66820, -0.12097, -9.77155, 0.49604),
        (-0.33606, -21.51688, 0.03178, -19.13444, -0.05028, -20.86477),
    ]

    devmgr = DeviceManager()
    devmgr.frequency = 100

    sa_offset = [-10, -9, 33, 21, 9, 11]

    cal = LoadcellCalibration(loadcell_matrix=decoupling_matrix, yaw=np.deg2rad(-30))
    sa = FlexSEAStrainAmp(
        bus="/dev/i2c-1",
        I2C_addr=0x66,
        amplifier_input_offset=sa_offset,
        calibration=cal,
        log_level="DEBUG",
    )
    with devmgr:
        zero_data = np.zeros(6)
        n_sampled = 0
        for tick in devmgr.clock:
            devmgr.update()

            # print(f"Fx: {sa.fx:.5f} N Fy: {sa.fy:.2f} N Fz: {sa.fz:.2f} N")
            # print(f"Mx: {sa.mx:.2f} Nm My: {sa.my:.2f} Nm Mz: {sa.mz:.2f} Nm")
