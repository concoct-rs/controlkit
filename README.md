<div align="center">
  <h1>Controlkit</h1>

 <a href="https://crates.io/crates/controlkit">
    <img src="https://img.shields.io/crates/v/controlkit?style=flat-square"
    alt="Crates.io version" />
  </a>
  <a href="https://docs.rs/controlkit">
    <img src="https://img.shields.io/badge/docs-latest-blue.svg?style=flat-square"
      alt="docs.rs docs" />
  </a>
   <a href="https://github.com/concoct-rs/controlkit/actions">
    <img src="https://github.com/concoct-rs/concoct/actions/workflows/rust.yml/badge.svg"
      alt="CI status" />
  </a>
</div>

<br />

Controlkit is a model-based robotics framework.

```rust
use concoct::{task::Task, System};
use controlkit::{pid_controller, PendulumPlant};

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
```