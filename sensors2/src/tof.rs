use crate::wait_ok;
use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Write, WriteRead},
    digital::v2::OutputPin,
};
use nb::block;
use uom::si::{f32::Length, length::meter};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Waiting,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VL6180XError;

pub struct VL6180X<T>
where
    T: OutputPin,
{
    enable_pin: T,
    device_address: u8,
    state: State,
}

impl<T> VL6180X<T>
where
    T: OutputPin,
{
    //register addresses
    const SYSTEM_INTERRUPT_CLEAR: u16 = 0x0015;
    const SYSTEM_FRESH_OUT_OF_RESET: u16 = 0x0016;
    const SYSTEM_INTERRUPT_CONFIG_GPIO: u16 = 0x0014;
    const SYSRANGE_START: u16 = 0x0018;
    const RESULT_INTERRUPT_STATUS_GPIO: u16 = 0x004F;
    const RESULT_RANGE_VAL: u16 = 0x0062;
    const I2C_SLAVE_DEVICE_ADDRESS: u16 = 0x0212;

    //default address of VL6180X
    const DEFAULT_ADDRESS: u8 = 0x29;

    //value of SYSTEM_FRESH_OUT_OF_RESET register in idle mode
    const IDLE_VALUE: u8 = 0x01;

    pub fn new<I: Write + WriteRead, V: DelayMs<u32>>(
        i2c: &'_ mut I,
        enable_pin: T,
        delay: &'_ mut V,
        new_address: u8, //7bit address
    ) -> Self {
        let mut tof = Self {
            enable_pin,
            device_address: Self::DEFAULT_ADDRESS,
            state: State::Idle,
        };

        tof.init(i2c, delay, new_address);

        tof
    }

    pub fn init<I: Write + WriteRead, V: DelayMs<u32>>(
        &mut self,
        i2c: &'_ mut I,
        delay: &'_ mut V,
        new_address: u8,
    ) {
        wait_ok!(self.enable_pin.set_low());

        wait_ok!(self.enable_pin.set_high()); //enable vl6180x

        delay.delay_ms(1); //wait for waking up

        wait_ok!(block!(self.check_if_working(i2c)));

        wait_ok!(self.configure_tuning(i2c));

        wait_ok!(self.configure_range_mode(i2c));

        wait_ok!(self.update_device_address(i2c, new_address));
    }

    fn update_device_address<I: Write>(
        &mut self,
        i2c: &'_ mut I,
        new_address: u8,
    ) -> Result<(), VL6180XError> {
        self.write_to_register(i2c, Self::I2C_SLAVE_DEVICE_ADDRESS, new_address)?;
        self.device_address = new_address;
        Ok(())
    }

    fn check_if_working<I: WriteRead>(&mut self, i2c: &'_ mut I) -> nb::Result<(), VL6180XError> {
        let data = self.read_from_registers(i2c, Self::SYSTEM_FRESH_OUT_OF_RESET)?;
        if data == Self::IDLE_VALUE {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn configure_range_mode<I: Write>(&mut self, i2c: &'_ mut I) -> Result<(), VL6180XError> {
        //enable internal interrupt of range mode
        self.write_to_register(i2c, Self::SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)?;
        Ok(())
    }

    fn configure_tuning<I: Write>(&mut self, i2c: &'_ mut I) -> Result<(), VL6180XError> {
        self.write_to_register(i2c, 0x0207, 0x01)?;
        self.write_to_register(i2c, 0x0208, 0x01)?;
        self.write_to_register(i2c, 0x0133, 0x01)?;
        self.write_to_register(i2c, 0x0096, 0x00)?;
        self.write_to_register(i2c, 0x0097, 0xFD)?;
        self.write_to_register(i2c, 0x00e3, 0x00)?;
        self.write_to_register(i2c, 0x00e4, 0x04)?;
        self.write_to_register(i2c, 0x00e5, 0x02)?;
        self.write_to_register(i2c, 0x00e6, 0x01)?;
        self.write_to_register(i2c, 0x00e7, 0x03)?;
        self.write_to_register(i2c, 0x00f5, 0x02)?;
        self.write_to_register(i2c, 0x00D9, 0x05)?;
        self.write_to_register(i2c, 0x00DB, 0xCE)?;
        self.write_to_register(i2c, 0x00DC, 0x03)?;
        self.write_to_register(i2c, 0x00DD, 0xF8)?;
        self.write_to_register(i2c, 0x009f, 0x00)?;
        self.write_to_register(i2c, 0x00a3, 0x3c)?;
        self.write_to_register(i2c, 0x00b7, 0x00)?;
        self.write_to_register(i2c, 0x00bb, 0x3c)?;
        self.write_to_register(i2c, 0x00b2, 0x09)?;
        self.write_to_register(i2c, 0x00ca, 0x09)?;
        self.write_to_register(i2c, 0x0198, 0x01)?;
        self.write_to_register(i2c, 0x01b0, 0x17)?;
        self.write_to_register(i2c, 0x01ad, 0x00)?;
        self.write_to_register(i2c, 0x00FF, 0x05)?;
        self.write_to_register(i2c, 0x0100, 0x05)?;
        self.write_to_register(i2c, 0x0199, 0x05)?;
        self.write_to_register(i2c, 0x0109, 0x07)?;
        self.write_to_register(i2c, 0x010a, 0x30)?;
        self.write_to_register(i2c, 0x003f, 0x46)?;
        self.write_to_register(i2c, 0x01a6, 0x1b)?;
        self.write_to_register(i2c, 0x01ac, 0x3e)?;
        self.write_to_register(i2c, 0x01a7, 0x1f)?;
        self.write_to_register(i2c, 0x0103, 0x01)?;
        self.write_to_register(i2c, 0x0030, 0x00)?;
        self.write_to_register(i2c, 0x001b, 0x0A)?;
        self.write_to_register(i2c, 0x003e, 0x0A)?;
        self.write_to_register(i2c, 0x0131, 0x04)?;
        self.write_to_register(i2c, 0x0011, 0x10)?;
        self.write_to_register(i2c, 0x0014, 0x24)?;
        self.write_to_register(i2c, 0x0031, 0xFF)?;
        self.write_to_register(i2c, 0x00d2, 0x01)?;
        self.write_to_register(i2c, 0x00f2, 0x01)?;
        Ok(())
    }

    fn write_to_register<I: Write>(
        &mut self,
        i2c: &'_ mut I,
        address: u16,
        data: u8,
    ) -> Result<(), VL6180XError> {
        let (address_h, address_l) = self.split_register_address(address);
        i2c.write(self.device_address, &[address_h, address_l, data])
            .map_err(|_| VL6180XError)?;
        Ok(())
    }

    fn read_from_registers<I: WriteRead>(
        &mut self,
        i2c: &'_ mut I,
        address: u16,
    ) -> Result<u8, VL6180XError> {
        let (address_h, address_l) = self.split_register_address(address);
        let mut buffer = [0];
        i2c.write_read(self.device_address, &[address_h, address_l], &mut buffer)
            .map_err(|_| VL6180XError)?;
        Ok(buffer[0])
    }

    fn split_register_address(&self, address: u16) -> (u8, u8) {
        //(higher,lower)
        ((address >> 8) as u8, address as u8)
    }

    pub fn distance<I: Write + WriteRead>(
        &mut self,
        i2c: &'_ mut I,
    ) -> nb::Result<Length, VL6180XError> {
        use State::*;

        match self.state {
            Idle => {
                //start polling
                self.write_to_register(i2c, Self::SYSRANGE_START, 0x01)?;
                self.state = Waiting;
                Err(nb::Error::WouldBlock)
            }
            Waiting => {
                let data = self.read_from_registers(i2c, Self::RESULT_INTERRUPT_STATUS_GPIO)?;
                if data & 0x04 != 0x04 {
                    return Err(nb::Error::WouldBlock);
                }
                //get result
                let distance = self.read_from_registers(i2c, Self::RESULT_RANGE_VAL)?;

                //teach VL6180X to finish polling
                self.write_to_register(i2c, Self::SYSTEM_INTERRUPT_CLEAR, 0x07)?;

                self.state = Idle;

                Ok(Length::new::<meter>(distance as f32 / 1000.0))
            }
        }
    }
}
