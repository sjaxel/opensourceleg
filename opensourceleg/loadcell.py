from typing import Union

import time
from abc import abstractmethod
from dataclasses import dataclass, field

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

    @property
    @abstractmethod
    def loadcell_data(self) -> [float]:
        pass

    @property
    @abstractmethod
    def cal(self) -> "LoadcellCalibration":
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
    loadcell_zero: np.ndarray = field(
        default=None
    )  #: loadcell zero offset (1 x nChannels)
    exc: float = field(default=5)  # excitation voltage of the strain guague

    def __post_init__(self):
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
            if self.loadcell_zero.shape != (1, self.nChannels):
                raise ValueError(
                    f"Loadcell zero for nChannels={self.nChannels} \
                    must be of shape ({self.nChannels},) or (1, {self.nChannels})"
                )

    def apply(self, loadcell_output: np.ndarray) -> np.ndarray:
        """Apply the calibration matrix and zero offset to the loadcell output

        This takes the raw loadcell output from an OUT = [Ch0, Ch1, ... ChN] and
        converts it to mV/V by multiplying by 1000*EXC (excitation voltage).

        Then, the loadcell calibration matrix is applied to convert the loadcell
        output to a wrench in N and Nm.

        Finally, any zero offset is subtracted from the wrench.

        Args:
            loadcell_output (np.ndarray): loadcell output (1 x nChannels) in units of V

        """
        if self.loadcell_zero is None:
            return np.transpose(self.loadcell_matrix.dot(np.transpose(loadcell_output)))
        else:
            return (
                np.transpose(self.loadcell_matrix.dot(np.transpose(loadcell_output)))
                - self.loadcell_zero
            )


class StrainAmp(OSLDevice, Loadcell):

    cal: LoadcellCalibration

    def __init__(self, name: str = "StrainAmp", **kwargs) -> None:
        super().__init__(name=name, **kwargs)


class FlexSEAStrainAmp(StrainAmp):
    """
    A class to directly manage the 6ch strain gauge amplifier over I2C.
    Author: Mitry Anderson
    """

    nCHANNELS: int = 6
    ADC_GAIN: int = 125  # gain of the strain amplifier
    ADC_FS: int = 2**12  # full scale of the strain amplifier (4096)
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
        calibration: LoadcellCalibration = None,
        **kwargs,
    ) -> None:
        super().__init__(name=name, **kwargs)
        self.bus = bus
        self.addr = I2C_addr

        if calibration is None:
            self._adc_datacal = LoadcellCalibration(
                nChannels=self.nCHANNELS, exc=self.AMP_EXC
            )
        else:
            self._cal = calibration
            self._cal.exc = self.AMP_EXC

        self._adc_data: np.ndarray
        self.output_data: np.ndarray = np.zeros(self.nCHANNELS)
        self.failed_reads = 0

    def read_compressed_strain(self):
        """Read the compressed adc data from the strain amp

        Used for more recent versions of strain amp firmware

        Raises:
            Exception: If the strain amp fails to respond after 5 attempts

        """
        try:
            i2c_data = self._SMBus.read_i2c_block_data(self.addr, self.MEM_R_CH1_H, 10)
            self.failed_reads = 0
        except OSError as e:
            self.failed_reads += 1
            # print("\n read failed")
            if self.failed_reads >= 5:
                raise OSError("Load cell unresponsive.")
            self._log.warning(
                f"Failed to read from strain amp with {self.failed_reads} attempts"
            )

        self.unpack_compressed_strain(i2c_data)

    def _start(self):
        self._SMBus = SMBus(self.bus)

    def _update(self):
        self.read_compressed_strain()
        self._calculate_strain_data()

    def _stop(self):
        self._SMBus.close()

    def unpack_compressed_strain(self, data: list[int]):
        """Used for more recent versions of strainamp firmware"""
        # ch1 = (data[0] << 4) | ( (data[1] >> 4) & 0x0F)
        # ch2 = ( (data[1] << 8) & 0x0F00) | data[2]
        # ch3 = (data[3] << 4) | ( (data[4] >> 4) & 0x0F)
        # ch4 = ( (data[4] << 8) & 0x0F00) | data[5]
        # ch5 = (data[6] << 4) | ( (data[7] >> 4) & 0x0F)
        # ch6 = ( (data[7] << 8) & 0x0F00) | data[8]
        # moved into one line to save 0.02ms -- maybe pointless but eh
        self._adc_data = np.array(
            [
                (data[0] << 4) | ((data[1] >> 4) & 0x0F),
                ((data[1] << 8) & 0x0F00) | data[2],
                (data[3] << 4) | ((data[4] >> 4) & 0x0F),
                ((data[4] << 8) & 0x0F00) | data[5],
                (data[6] << 4) | ((data[7] >> 4) & 0x0F),
                ((data[7] << 8) & 0x0F00) | data[8],
            ]
        )

    def _calculate_strain_data(self):
        """Converts strain values between 0 and 4095 to a wrench in N and Nm

        TODO: This function should be split between the channels and moved to the
        property getters to avoid doing unnecessary calculations when strain
        is not accessed.
        """
        loadcell_code_signed = self._adc_data - (self.ADC_FS / 2)  # To signed code
        V_adc = loadcell_code_signed * (self.ADC_VREF / (self.ADC_FS))  # Code to V_adc
        mV_out = V_adc * 1000 / self.ADC_GAIN  # V_adc to mV_out
        self.output_data = self.cal.apply(mV_out)  # Apply decoupling and zero offset

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

    devmgr = DeviceManager(frequency=10)

    cal = LoadcellCalibration(loadcell_matrix=decoupling_matrix)
    sa = FlexSEAStrainAmp(bus="/dev/i2c-1", I2C_addr=0x66, calibration=cal)
    with devmgr:
        for tick in devmgr.clock:
            devmgr.update()
            ##print(f"Fx: {sa.fx:.5f} N Fy: {sa.fy:.2f} N Fz: {sa.fz:.2f} N")
            time.sleep(0.1)
