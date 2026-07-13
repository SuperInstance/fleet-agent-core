// fleet-agent-core — Demonstration
//
// Runs a single FleetAgent for 100 ticks with simulated observations.
// Prints phase transitions and actions taken.

use fleet_agent_core::*;

fn main() {
    println!("=== Fleet Agent Core — 100 Tick Demonstration ===\n");

    let config = AgentConfig {
        id: "forgemaster-1".into(),
        keel_date: 1700000000,
        constraints: vec![
            Constraint {
                name: "temperature".into(),
                description: "Core temp must stay below 85.0".into(),
                threshold: 85.0,
            },
            Constraint {
                name: "vibration".into(),
                description: "Vibration amplitude below 20.0".into(),
                threshold: 20.0,
            },
        ],
        heading: Some(Heading {
            direction: vec![1.0, 0.0, 0.0],
            scope: vec!["north".into(), "up".into()],
        }),
        coupling: 0.3,
        gain: 1.5,
    };

    let mut agent = FleetAgent::new(config);
    let mut prev_phase: Option<Phase> = None;

    for tick in 0..100 {
        // Simulate external observations — stress event introduced at tick 55
        let external = if tick == 55 {
            vec![Observation {
                from: "drone-7".into(),
                state: State {
                    values: vec![0.9],
                    timestamp: 1700000000 + tick,
                    sign_pattern: vec![1],
                },
                bearing_rate: 0.001, // collision course!
            }]
        } else if tick % 10 == 0 {
            vec![Observation {
                from: format!("drone-{}", tick % 5),
                state: State {
                    values: vec![0.5 + (tick as f64 * 0.01)],
                    timestamp: 1700000000 + tick,
                    sign_pattern: vec![1],
                },
                bearing_rate: 0.5 + (tick as f64 * 0.005),
            }]
        } else {
            vec![]
        };

        let actions = agent.tick(external);
        let phase = agent.phase();

        // Print phase transitions
        if prev_phase != Some(phase) {
            println!(
                "  tick {:>3} | ▶ phase transition: {} → {}",
                tick,
                prev_phase
                    .map(|p| p.to_string())
                    .unwrap_or_else(|| "—".into()),
                phase
            );
            prev_phase = Some(phase);
        }

        // Print non-Hold actions
        for a in &actions {
            if *a != Action::Hold {
                println!("  tick {:>3} |   action: {}", tick, a);
            }
        }
    }

    println!();
    println!("=== Summary ===");
    println!("  Final phase:          {}", agent.phase());
    println!("  Total ticks:          {}", agent.age());
    println!("  Log entries:          {}", agent.log().len());
    println!("  Constraint check:     {}", agent.check_constraints());
    println!(
        "  Build refits:         {}",
        agent.build_record().refits.len()
    );
    println!(
        "  Build prunes:         {}",
        agent.build_record().prunes.len()
    );

    // Phase distribution
    let mut counts = std::collections::HashMap::new();
    for entry in agent.log() {
        *counts.entry(entry.phase).or_insert(0) += 1;
    }
    println!("  Phase distribution:");
    for (phase, count) in &counts {
        println!("    {:16}: {}", format!("{}", phase), count);
    }

    println!();
    println!("=== One-loop Architecture ===");
    println!("  SENSE → CONSTRAINT CHECK → ORIENT → DECIDE → ACT → RECORD → repeat");
}
