use std::{
    marker::PhantomData,
    time::{Duration, Instant},
};

pub trait Task<M> {
    type State;

    fn build(&mut self, model: &mut M) -> Self::State;

    fn rebuild(&mut self, model: &mut M, state: &mut Self::State);
}

impl<M> Task<M> for () {
    type State = ();

    fn build(&mut self, model: &mut M) -> Self::State {}

    fn rebuild(&mut self, model: &mut M, state: &mut Self::State) {}
}

pub fn pid_controller<M, F, T>(
    value: f64,
    target: f64,
    kp: f64,
    ki: f64,
    kd: f64,
    f: F,
) -> PidController<F, M, T>
where
    F: FnMut(&mut M, f64) -> T,
    T: Task<M>,
{
    PidController {
        f,
        value,
        target,
        kp,
        ki,
        kd,
        _marker: PhantomData,
    }
}

pub struct State<T> {
    total_error: f64,
    last_error: f64,
    last_instant: Option<Instant>,
    inner: T,
}

pub struct PidController<F, M, T> {
    f: F,
    kp: f64,
    ki: f64,
    kd: f64,
    value: f64,
    target: f64,
    _marker: PhantomData<(M, T)>,
}

impl<F, M, T> PidController<F, M, T> {}

impl<M, F, T> Task<M> for PidController<F, M, T>
where
    F: FnMut(&mut M, f64) -> T,
    T: Task<M>,
{
    type State = State<T::State>;

    fn build(&mut self, model: &mut M) -> Self::State {
        let out = self.kp * (self.target - self.value);
        let mut task = (self.f)(model, out);
        let inner = task.build(model);

        State {
            total_error: 0.,
            last_error: 0.,
            last_instant: None,
            inner,
        }
    }

    fn rebuild(&mut self, model: &mut M, state: &mut Self::State) {
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
        let out = p + i + d;

        let mut task = (self.f)(model, out);
        task.rebuild(model, &mut state.inner)
    }
}
