use std::{
    marker::PhantomData,
    time::{Duration, Instant},
};

pub trait Task<M> {
    type Output;

    type State;

    fn build(&mut self, model: &mut M) -> (Self::Output, Self::State);

    fn rebuild(&mut self, model: &mut M, state: &mut Self::State) -> Self::Output;

    fn then<F, T>(self, f: F) -> Then<Self, F, T, M>
    where
        Self: Sized + 'static,
        F: FnMut(&mut M, Self::Output) -> T + 'static,
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
    type Output = ();
    
    type State = ();

    fn build(&mut self, model: &mut M) -> (Self::Output, Self::State) {
        ((), ())
    }

    fn rebuild(&mut self, model: &mut M, state: &mut Self::State) -> Self::Output {
      
    }

    
}

pub struct Then<T1, F, T2, M> {
    task: T1,
    f: F,
    _marker: PhantomData<(T2, M)>,
}

impl<T1, F, T2, M> Task<M> for Then<T1, F, T2, M>
where
    T1: Task<M> + 'static,
    F: FnMut(&mut M, T1::Output) -> T2 + 'static,
    T2: Task<M> + 'static,
    M: 'static,
{
    type Output = T2::Output;

    type State = (T1::State, T2::State);
    

   fn build(&mut self, model: &mut M) -> (Self::Output, Self::State) {
   
        let  (output, mut state) = self.task.build(model);
        let mut next = (self.f)(model, output);
        let (next_output, next_state) = next.build(model);
        (next_output, (state, next_state))

   }

    fn rebuild(&mut self, model: &mut M, state: &mut Self::State) -> Self::Output {
       let output= self.task.rebuild(model, &mut state.0);
        let mut next = (self.f)(model, output);
        next.rebuild(model, &mut state.1)
    }
}

pub fn pid_controller(
    value: f64,
    target: f64,
    kp: f64,
    ki: f64,
    kd: f64,
) -> PidController

{
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


impl<M> Task<M> for PidController
{
    type Output = f64;
    type State = PidControllerState;

    fn build(&mut self, model: &mut M) -> (Self::Output,Self::State) {
    
        (0., PidControllerState {
            total_error: 0.,
            last_error: 0.,
            last_instant: None,
            
        })
    }

    fn rebuild(&mut self, model: &mut M, state: &mut Self::State) -> Self::Output {
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
    type Output = f64;

    type State = PendulumState;

    fn build(&mut self, model: &mut M) -> (Self::Output, Self::State ){
        (0.,PendulumState {
            angle: 0.,
            angular_velocity: 1.,
            last_instant: Instant::now(),
        })
    }

    fn rebuild(&mut self, model: &mut M, state: &mut Self::State)-> Self::Output {
        let elapsed = Instant::now() - state.last_instant;
        state.last_instant = Instant::now();

        let elapsed_ms = (elapsed.as_millis() as f64).max(1.0);
        self.update(state, elapsed_ms);
        state.angle
    }
}
