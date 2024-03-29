use core::marker::PhantomData;

use embedded_hal::adc::{Channel, OneShot};
use nb::block;
use uom::si::{
    electric_potential::volt,
    f32::{ElectricPotential, Frequency, Time},
    ratio::ratio,
};

use super::motor::Voltmeter as IVoltmeter;

pub struct Voltmeter<T, ADC, PIN>
where
    T: OneShot<ADC, u16, PIN>,
    PIN: Channel<ADC>,
{
    adc: T,
    adc_pin: PIN,
    voltage: ElectricPotential,
    alpha: f32,
    _adc_marker: PhantomData<ADC>,
}

impl<T, ADC, PIN> Voltmeter<T, ADC, PIN>
where
    T: OneShot<ADC, u16, PIN>,
    PIN: Channel<ADC>,
    <T as OneShot<ADC, u16, PIN>>::Error: core::fmt::Debug,
{
    const RATIO: f32 = 1.9;
    const AVDD_VOLTAGE: ElectricPotential = ElectricPotential {
        dimension: PhantomData,
        units: PhantomData,
        value: 3.3,
    };
    const MAX_ADC_VALUE: f32 = 4096.0;
    const SUM_NUM: u16 = 100;

    pub fn new(adc: T, adc_pin: PIN, period: Time, cut_off_frequency: Frequency) -> Self {
        let alpha =
            1.0 / (2.0 * core::f32::consts::PI * (period * cut_off_frequency).get::<ratio>() + 1.0);
        let mut voltmeter = Self {
            adc,
            adc_pin,
            voltage: ElectricPotential::new::<volt>(0.0),
            alpha,
            _adc_marker: PhantomData,
        };

        let mut sum = ElectricPotential::default();
        for _ in 0..Self::SUM_NUM {
            sum += voltmeter.get_current_voltage();
        }
        voltmeter.voltage = sum / Self::SUM_NUM as f32;
        voltmeter
    }

    #[allow(unused)]
    pub fn update_voltage(&mut self) {
        self.voltage = self.alpha * self.voltage + (1.0 - self.alpha) * self.get_current_voltage();
    }

    pub fn get_current_voltage(&mut self) -> ElectricPotential {
        let value = block!(self.adc.read(&mut self.adc_pin)).unwrap() as f32;
        value * Self::AVDD_VOLTAGE * Self::RATIO / Self::MAX_ADC_VALUE
    }
}

impl<T, ADC, PIN> IVoltmeter for Voltmeter<T, ADC, PIN>
where
    T: OneShot<ADC, u16, PIN>,
    PIN: Channel<ADC>,
    <T as OneShot<ADC, u16, PIN>>::Error: core::fmt::Debug,
{
    fn get_voltage(&mut self) -> ElectricPotential {
        // self.update_voltage();
        self.voltage
    }
}
