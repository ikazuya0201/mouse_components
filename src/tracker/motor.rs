use uom::si::f32::ElectricPotential;

pub trait Motor {
    fn apply(&mut self, electric_potential: ElectricPotential);
}
