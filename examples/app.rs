use botkit::{pid_controller, PendulumPlant, System, Task};

#[derive(Default)]
struct Model {
    state: f64,
}

fn app(model: &mut Model) -> impl Task<Model> {
    pid_controller(model.state, 0., 0.5, 0.1, 0.2)
        .then(|_, control| PendulumPlant::new(control))
        .then(|model: &mut Model, angle| {
            model.state = angle;
        })
}

fn main() {
    let mut system = System::new(Model::default(), app);
    system.build();

    for _ in 0..10000 {
        system.rebuild();
        dbg!(system.model.state);
    }
}
