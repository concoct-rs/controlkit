use std::{
    marker::PhantomData,
    time::{Duration, Instant},
};

pub trait Task<M> {
    type State;

    fn build(&mut self, model: &mut M) -> Self::State;

    fn rebuild(&mut self, model: &mut M, state: &mut Self::State);

    fn then<F, T>(self, f: F) -> Then<Self, F, T, M>
    where
        Self: Sized + 'static,
        F: FnMut(&mut M, &mut Self::State) -> T + 'static,
        T: Task<M> + 'static,
        M: 'static,
    {
        Then {
            task: self,
            f,
            _marker: PhantomData,
        }
    }
}

impl<M> Task<M> for () {
    type State = ();

    fn build(&mut self, model: &mut M) -> Self::State {}

    fn rebuild(&mut self, model: &mut M, state: &mut Self::State) {}
}

pub struct Then<T1, F, T2, M> {
    task: T1,
    f: F,
    _marker: PhantomData<(T2, M)>,
}

impl<T1, F, T2, M> Task<M> for Then<T1, F, T2, M>
where
    T1: Task<M> + 'static,
    F: FnMut(&mut M, &mut T1::State) -> T2 + 'static,
    T2: Task<M> + 'static,
    M: 'static,
{
    type State = (T1::State, T2::State);

    fn build(&mut self, model: &mut M) -> Self::State {
        let mut state = self.task.build(model);
        let mut next = (self.f)(model, &mut state);
        let next_state = next.build(model);
        (state, next_state)
    }

    fn rebuild(&mut self, model: &mut M, state: &mut Self::State) {
        self.task.rebuild(model, &mut state.0);
        let mut next = (self.f)(model, &mut state.0);
        next.rebuild(model, &mut state.1);
    }
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

pub struct PidControllerState<T> {
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
    type State = PidControllerState<T::State>;

    fn build(&mut self, model: &mut M) -> Self::State {
        let out = self.kp * (self.target - self.value);
        let mut task = (self.f)(model, out);
        let inner = task.build(model);

        PidControllerState {
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

pub struct PendulumState {
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
    fn update(&mut self, state: &mut PendulumState, dt: f64) {
        let angular_acceleration = (-self.gravity / self.length * state.angle.sin()
            - self.damping * state.angular_velocity
            + self.torque)
            / self.length;

        state.angular_velocity += angular_acceleration * dt;
        state.angle += state.angular_velocity * dt;
    }
}

impl<M> Task<M> for PendulumPlant {
    type State = PendulumState;

    fn build(&mut self, model: &mut M) -> Self::State {
        PendulumState {
            angle: 0.,
            angular_velocity: 1.,
            last_instant: Instant::now(),
        }
    }

    fn rebuild(&mut self, model: &mut M, state: &mut Self::State) {
        let elapsed = Instant::now() - state.last_instant;
        state.last_instant = Instant::now();

        let elapsed_ms = (elapsed.as_millis() as f64).max(1.0);
        self.update(state, elapsed_ms);
        dbg!(state.angular_velocity);
    }
}
