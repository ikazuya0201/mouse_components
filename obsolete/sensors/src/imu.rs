use core::convert::Infallible;
use core::marker::PhantomData;

use embedded_hal::{
    blocking::delay::DelayMs, blocking::spi::Transfer, digital::v2::OutputPin, timer::CountDown,
};
use mousecore::sensors::Imu;
use nb::block;
use uom::si::f32::{Acceleration, AngularVelocity};

use crate::wait_ok;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImuError;

impl From<Infallible> for ImuError {
    fn from(_error: Infallible) -> Self {
        Self
    }
}

pub struct ICM20648<T, U> {
    spi: T,
    cs: U,
    accel_offset: Acceleration,
    gyro_offset: AngularVelocity,
}

impl<T, U> ICM20648<T, U>
where
    T: Transfer<u8>,
    U: OutputPin,
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

    pub fn new<'a, V, W, F>(spi: T, cs: U, delay: &'a mut V, timer: &'a mut W, post_init: F) -> Self
    where
        V: DelayMs<u32>,
        W: CountDown,
        F: Fn(T) -> T,
    {
        let mut icm = Self {
            spi,
            cs,
            accel_offset: Default::default(),
            gyro_offset: Default::default(),
        };

        icm.init(delay, timer);

        icm.spi = (post_init)(icm.spi);

        icm
    }

    pub fn init<'a, V, W>(&mut self, delay: &'a mut V, timer: &'a mut W)
    where
        V: DelayMs<u32>,
        W: CountDown,
    {
        wait_ok!(self.write_to_register(Self::RA_PWR_MGMT_1, 0x80)); //reset icm20648

        delay.delay_ms(10); //wait while reset

        wait_ok!(block!(self.check_who_am_i())); //wait for who am i checking

        let mut write = |register: u8, value: u8| {
            delay.delay_ms(1);
            wait_ok!(self.write_to_register(register, value));
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

        wait_ok!(self.calibrate(timer));
        wait_ok!(self.calibrate(timer));
    }

    pub fn calibrate<W>(&mut self, timer: &'_ mut W) -> Result<(), ImuError>
    where
        W: CountDown,
    {
        let mut accel_offset_sum = Acceleration::default();
        let mut gyro_offset_sum = AngularVelocity::default();
        for _ in 0..Self::CALIBRATION_NUM {
            let accel = block!(self.get_translational_acceleration())?;
            let gyro = block!(self.get_angular_velocity())?;
            accel_offset_sum += accel;
            gyro_offset_sum += gyro;
            block!(timer.wait()).ok();
        }
        self.accel_offset += accel_offset_sum / Self::CALIBRATION_NUM as f32;
        self.gyro_offset += gyro_offset_sum / Self::CALIBRATION_NUM as f32;
        Ok(())
    }

    fn check_who_am_i(&mut self) -> nb::Result<(), ImuError> {
        let mut buffer = [0; 2];
        let buffer = self.read_from_registers(Self::RA_WHO_AM_I, &mut buffer)?;
        if buffer[0] == Self::ICM20648_DEVICE_ID {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn assert(&mut self) -> Result<(), ImuError> {
        self.cs.set_low().map_err(|_| ImuError)
    }

    fn deassert(&mut self) -> Result<(), ImuError> {
        self.cs.set_high().map_err(|_| ImuError)
    }

    fn write_to_register(&mut self, address: u8, data: u8) -> Result<(), ImuError> {
        self.assert()?;
        let res = self._write_to_register(address, data);
        self.deassert()?;
        res
    }

    fn _write_to_register(&mut self, address: u8, data: u8) -> Result<(), ImuError> {
        self.spi
            .transfer(&mut [address, data])
            .map_err(|_| ImuError)?;
        Ok(())
    }

    //size of buffer should be equal to {data length}+1
    fn read_from_registers<'w>(
        &mut self,
        address: u8,
        buffer: &'w mut [u8],
    ) -> Result<&'w [u8], ImuError> {
        self.assert()?;
        let res = self._read_from_registers(address, buffer);
        self.deassert()?;
        res
    }

    fn _read_from_registers<'w>(
        &mut self,
        address: u8,
        buffer: &'w mut [u8],
    ) -> Result<&'w [u8], ImuError> {
        buffer[0] = address | 0x80;
        let buffer = self.spi.transfer(buffer).map_err(|_| ImuError)?;
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
}

impl<T, U> Imu for ICM20648<T, U>
where
    T: Transfer<u8>,
    U: OutputPin,
{
    type Error = ImuError;

    fn get_angular_velocity(&mut self) -> nb::Result<AngularVelocity, Self::Error> {
        let mut buffer = [0; 3];
        let buffer = self.read_from_registers(Self::RA_GYRO_Z_OUT_H, &mut buffer)?;
        Ok(
            self.convert_raw_data_to_angular_velocity(self.connect_raw_data(buffer[0], buffer[1]))
                - self.gyro_offset,
        )
    }

    fn get_translational_acceleration(&mut self) -> nb::Result<Acceleration, Self::Error> {
        let mut buffer = [0; 3];
        let buffer = self.read_from_registers(Self::RA_ACCEL_Y_OUT_H, &mut buffer)?;
        Ok(
            -self.convert_raw_data_to_acceleration(self.connect_raw_data(buffer[0], buffer[1]))
                - self.accel_offset,
        )
    }
}
