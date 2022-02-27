use core::convert::Infallible;
use core::marker::PhantomData;

use embedded_hal::{
    blocking::delay::DelayMs, blocking::spi::Transfer, digital::v2::OutputPin, timer::CountDown,
};
use nb::block;
use uom::si::f32::{Acceleration, AngularVelocity};

use crate::wait_ok;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ICM20648Error;

impl From<Infallible> for ICM20648Error {
    fn from(_error: Infallible) -> Self {
        Self
    }
}

pub struct ICM20648<T> {
    cs: T,
    accel_offset: Acceleration,
    gyro_offset: AngularVelocity,
}

impl<T> ICM20648<T>
where
    T: OutputPin,
{
    //RA: register address
    //all bank
    const RA_REG_BANK_SEL: u8 = 0x7F;
    //user bank 0
    const RA_WHO_AM_I: u8 = 0x00;
    const RA_LP_CONFIG: u8 = 0x05;
    const RA_PWR_MGMT_1: u8 = 0x06;
    //gyrometer
    const RA_GYRO_Z_OUT_H: u8 = 0x37;
    //accelerometer
    const RA_ACCEL_Y_OUT_H: u8 = 0x2F;
    //user bank 2
    const RA_GYRO_CONFIG_1: u8 = 0x01;
    const RA_ACCEL_CONFIG: u8 = 0x14;

    const ICM20648_DEVICE_ID: u8 = 0xE0;

    const GYRO_SENSITIVITY_SCALE_FACTOR: AngularVelocity = AngularVelocity {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.001_064_225_1,
    };
    const ACCEL_SENSITIVITY_SCALE_FACTOR: Acceleration = Acceleration {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.000_598_550_4,
    };

    const CALIBRATION_NUM: u16 = 1000;

    pub fn new<S, V, W>(spi: &mut S, cs: T, delay: &mut V, timer: &mut W) -> Self
    where
        S: Transfer<u8>,
        V: DelayMs<u32>,
        W: CountDown,
    {
        let mut icm = Self {
            cs,
            accel_offset: Default::default(),
            gyro_offset: Default::default(),
        };

        icm.init(spi, delay, timer);

        icm
    }

    pub fn init<S, V, W>(&mut self, spi: &mut S, delay: &mut V, timer: &mut W)
    where
        S: Transfer<u8>,
        V: DelayMs<u32>,
        W: CountDown,
    {
        wait_ok!(self.write_to_register(spi, Self::RA_PWR_MGMT_1, 0x80)); //reset icm20648

        delay.delay_ms(10); //wait while reset

        wait_ok!(block!(self.check_who_am_i(spi))); //wait for who am i checking

        let mut write = |register: u8, value: u8| {
            delay.delay_ms(1);
            wait_ok!(self.write_to_register(spi, register, value));
        };

        write(Self::RA_PWR_MGMT_1, 0x01);

        write(Self::RA_LP_CONFIG, 0x00); //disable duty cycle mode for gyro

        write(Self::RA_REG_BANK_SEL, 0x20); //switch to user bank 2

        //configure gryo to +-2000dps in full scale
        write(Self::RA_GYRO_CONFIG_1, 0x06);

        //disable digital low path filter
        //configure accelerometer to +-2g
        write(Self::RA_ACCEL_CONFIG, 0x00);

        write(Self::RA_REG_BANK_SEL, 0x00); //switch to user bank 0

        self.accel_offset = Default::default();
        self.gyro_offset = Default::default();
        wait_ok!(self.calibrate(spi, timer));
        wait_ok!(self.calibrate(spi, timer));
    }

    pub fn calibrate<S, W>(&mut self, spi: &mut S, timer: &mut W) -> Result<(), ICM20648Error>
    where
        W: CountDown,
        S: Transfer<u8>,
    {
        let mut accel_offset_sum = Acceleration::default();
        let mut gyro_offset_sum = AngularVelocity::default();
        for _ in 0..Self::CALIBRATION_NUM {
            let accel = block!(self.translational_acceleration(spi))?;
            let gyro = block!(self.angular_velocity(spi))?;
            accel_offset_sum += accel;
            gyro_offset_sum += gyro;
            block!(timer.wait()).ok();
        }
        self.accel_offset += accel_offset_sum / Self::CALIBRATION_NUM as f32;
        self.gyro_offset += gyro_offset_sum / Self::CALIBRATION_NUM as f32;
        Ok(())
    }

    fn check_who_am_i<S: Transfer<u8>>(&mut self, spi: &mut S) -> nb::Result<(), ICM20648Error> {
        let mut buffer = [0; 2];
        let buffer = self.read_from_registers(spi, Self::RA_WHO_AM_I, &mut buffer)?;
        if buffer[0] == Self::ICM20648_DEVICE_ID {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn assert(&mut self) -> Result<(), ICM20648Error> {
        self.cs.set_low().map_err(|_| ICM20648Error)
    }

    fn deassert(&mut self) -> Result<(), ICM20648Error> {
        self.cs.set_high().map_err(|_| ICM20648Error)
    }

    fn write_to_register<S: Transfer<u8>>(
        &mut self,
        spi: &mut S,
        address: u8,
        data: u8,
    ) -> Result<(), ICM20648Error> {
        self.assert()?;
        let res = Self::_write_to_register(spi, address, data);
        self.deassert()?;
        res
    }

    fn _write_to_register<S: Transfer<u8>>(
        spi: &mut S,
        address: u8,
        data: u8,
    ) -> Result<(), ICM20648Error> {
        spi.transfer(&mut [address, data])
            .map_err(|_| ICM20648Error)?;
        Ok(())
    }

    //size of buffer should be equal to {data length}+1
    fn read_from_registers<'w, S: Transfer<u8>>(
        &mut self,
        spi: &mut S,
        address: u8,
        buffer: &'w mut [u8],
    ) -> Result<&'w [u8], ICM20648Error> {
        self.assert()?;
        let res = Self::_read_from_registers(spi, address, buffer);
        self.deassert()?;
        res
    }

    fn _read_from_registers<'w, S: Transfer<u8>>(
        spi: &mut S,
        address: u8,
        buffer: &'w mut [u8],
    ) -> Result<&'w [u8], ICM20648Error> {
        buffer[0] = address | 0x80;
        let buffer = spi.transfer(buffer).map_err(|_| ICM20648Error)?;
        Ok(&buffer[1..])
    }

    #[inline]
    fn connect_raw_data(&self, higher: u8, lower: u8) -> i16 {
        ((higher as u16) << 8 | lower as u16) as i16
    }

    fn convert_raw_data_to_angular_velocity(&mut self, gyro_value: i16) -> AngularVelocity {
        Self::GYRO_SENSITIVITY_SCALE_FACTOR * gyro_value as f32 * 0.99
    }

    fn convert_raw_data_to_acceleration(&mut self, accel_value: i16) -> Acceleration {
        Self::ACCEL_SENSITIVITY_SCALE_FACTOR * accel_value as f32
    }

    pub fn angular_velocity<S: Transfer<u8>>(
        &mut self,
        spi: &mut S,
    ) -> nb::Result<AngularVelocity, ICM20648Error> {
        let mut buffer = [0; 3];
        let buffer = self.read_from_registers(spi, Self::RA_GYRO_Z_OUT_H, &mut buffer)?;
        Ok(
            self.convert_raw_data_to_angular_velocity(self.connect_raw_data(buffer[0], buffer[1]))
                - self.gyro_offset,
        )
    }

    pub fn translational_acceleration<S: Transfer<u8>>(
        &mut self,
        spi: &mut S,
    ) -> nb::Result<Acceleration, ICM20648Error> {
        let mut buffer = [0; 3];
        let buffer = self.read_from_registers(spi, Self::RA_ACCEL_Y_OUT_H, &mut buffer)?;
        Ok(
            -self.convert_raw_data_to_acceleration(self.connect_raw_data(buffer[0], buffer[1]))
                - self.accel_offset,
        )
    }
}
