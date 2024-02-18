use concoct::task::Task;
use std::time::Instant;

pub struct PendulumPlantState {
    pub angle: f64,
    pub angular_velocity: f64,
    pub last_instant: Instant,
}

pub struct PendulumPlant {
    length: f64,
    gravity: f64,
    damping: f64,
    torque: f64,
}

impl PendulumPlant {
    pub fn new(torque: f64) -> Self {
        Self {
            length: 10.,
            gravity: 9.81,
            damping: 0.5,
            torque,
        }
    }
}

impl PendulumPlant {
    fn update(&mut self, state: &mut PendulumPlantState, dt: f64) {
        let angular_acceleration = (-self.gravity / self.length * state.angle.sin()
            - self.damping * state.angular_velocity
            + self.torque)
            / self.length;

        state.angular_velocity += angular_acceleration * dt;
        state.angle += state.angular_velocity * dt;
    }
}

impl<M> Task<M> for PendulumPlant {
    type Output = f64;

    type State = PendulumPlantState;

    fn build(
        &mut self,
        _cx: &concoct::task::Context<M, ()>,
        _model: &mut M,
    ) -> (Self::Output, Self::State) {
        (
            0.,
            PendulumPlantState {
                angle: 0.,
                angular_velocity: 1.,
                last_instant: Instant::now(),
            },
        )
    }

    fn rebuild(
        &mut self,
        _cx: &concoct::task::Context<M, ()>,
        _model: &mut M,
        state: &mut Self::State,
    ) -> Self::Output {
        let elapsed = Instant::now() - state.last_instant;
        state.last_instant = Instant::now();

        let elapsed_ms = (elapsed.as_millis() as f64).max(1.0);
        self.update(state, elapsed_ms);
        state.angle
    }
}
