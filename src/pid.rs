use concoct::task::Task;
use std::time::{Duration, Instant};

pub fn pid_controller(value: f64, target: f64, kp: f64, ki: f64, kd: f64) -> PidController {
    PidController {
        value,
        target,
        kp,
        ki,
        kd,
    }
}

pub struct PidControllerState {
    pub total_error: f64,
    pub last_error: f64,
    pub last_instant: Option<Instant>,
}

pub struct PidController {
    kp: f64,
    ki: f64,
    kd: f64,
    value: f64,
    target: f64,
}

impl<M> Task<M> for PidController {
    type Output = f64;
    type State = PidControllerState;

    fn build(
        &mut self,
        _cx: &concoct::task::Context<M, ()>,
        _model: &mut M,
    ) -> (Self::Output, Self::State) {
        (
            0.,
            PidControllerState {
                total_error: 0.,
                last_error: 0.,
                last_instant: None,
            },
        )
    }

    fn rebuild(
        &mut self,
        _cx: &concoct::task::Context<M, ()>,
        _model: &mut M,
        state: &mut Self::State,
    ) -> Self::Output {
        let now = Instant::now();
        let elapsed = match state.last_instant {
            Some(last_time) => now.duration_since(last_time),
            None => Duration::from_millis(1),
        };
        let elapsed_ms = (elapsed.as_millis() as f64).max(1.0);

        let error = self.target - self.value;
        let error_delta = (error - state.last_error) / elapsed_ms;
        state.total_error += error * elapsed_ms;
        state.last_error = error;
        state.last_instant = Some(now);

        let p = self.kp * error;
        let i = self.ki * state.total_error;
        let d = self.kd * error_delta;
        p + i + d
    }
}
