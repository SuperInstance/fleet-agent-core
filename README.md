# Fleet Agent Core ⚒️

A single-binary fleet agent in Rust. One process. One event loop. Zero external dependencies except `std`.

## One-Loop Architecture

```
                    ┌─────────────────────────┐
                    │       RECORD            │
                    │  log tick, phase, action│
                    └─────────┬───────────────┘
                              │
                    ┌─────────▼───────────────┐
   external ──────▶ │         SENSE           │
   observations     │  filter relevant obs    │
                    └─────────┬───────────────┘
                              │
                    ┌─────────▼───────────────┐
                    │    CONSTRAINT CHECK      │
                    │  verify all constraints  │
                    └─────────┬───────────────┘
                              │
                    ┌─────────▼───────────────┐
                    │        ORIENT           │
                    │  determine phase        │
                    └─────────┬───────────────┘
                              │
                    ┌─────────▼───────────────┐
                    │        DECIDE           │
                    │  choose actions by phase│
                    └─────────┬───────────────┘
                              │
                    ┌─────────▼───────────────┐
                    │         ACT             │
                    │  execute side-effects   │
                    │  (placeholder: stderr)  │
                    └─────────┬───────────────┘
                              │
                              ▼
                        repeat next tick
```

## Phases

| Phase | Trigger | Behavior |
|---|---|---|
| **Commissioning** | `age < 50` ticks | Hold only — learn, don't act |
| **Operational** | normal conditions | Respond to observations, maintain heading |
| **Stressed** | `bearing_rate < 0.01` (collision course) | Evasive heading + Broadcast |
| **Recovering** | state energy `sum(abs(values)) < 0.5` | Hold for stability |

## Types

- `AgentId`, `AgentConfig` — identity and configuration
  - `coupling` and `gain` are reserved fields; not yet used by the loop
- `State` — vector of float values with sign pattern
- `Observation` — peer state + bearing rate
- `Action` — `Hold | ChangeHeading | Refit | Prune | Broadcast`
- `Phase` — `Commissioning | Operational | Stressed | Recovering`
- `BuildRecord` — keel date, refits, prunes (populated from `Refit`/`Prune` actions)
- `LogEntry` — (tick, phase, observation, action)

## Usage

```rust
use fleet_agent_core::*;

let config = AgentConfig {
    id: "forgemaster-1".into(),
    keel_date: 1700000000,
    constraints: vec![Constraint {
        name: "temperature".into(),
        description: "Core temp < 85.0".into(),
        threshold: 85.0,
    }],
    heading: Some(Heading {
        direction: vec![1.0, 0.0, 0.0],
        scope: vec!["north".into()],
    }),
    coupling: 0.3,
    gain: 1.5,
};

let mut agent = FleetAgent::new(config);

for tick in 0..100 {
    let observations = vec![/* external observations */];
    let actions = agent.tick(observations);
    println!("tick {} phase {} => {:?}", tick, agent.phase(), actions);
}
```

## Build & Test

```bash
cargo build
cargo test    # 35 tests
cargo run     # 100-tick demonstration
```

Continuous integration runs `cargo fmt --check`, `cargo clippy --all-targets --all-features -- -D warnings`, `cargo build`, and `cargo test` on every push and PR.

## License

MIT
