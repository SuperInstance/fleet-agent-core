// fleet-agent-core — one loop from metal to meaning
//
// Architecture:
//   SENSE → CONSTRAINT CHECK → ORIENT → DECIDE → ACT → RECORD → repeat

use std::fmt;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

pub type AgentId = String;

#[derive(Debug, Clone)]
pub struct AgentConfig {
    pub id: AgentId,
    pub keel_date: u64,
    pub constraints: Vec<Constraint>,
    pub heading: Option<Heading>,
    pub coupling: f64,
    pub gain: f64,
}

#[derive(Debug, Clone)]
pub struct Constraint {
    pub name: String,
    /// A human-readable description of the predicate
    pub description: String,
    /// Threshold above which the constraint is considered violated
    pub threshold: f64,
}

#[derive(Debug, Clone)]
pub struct State {
    pub values: Vec<f64>,
    pub timestamp: u64,
    pub sign_pattern: Vec<i8>,
}

impl State {
    /// Total energy: sum of absolute values (used for phase detection)
    pub fn energy(&self) -> f64 {
        self.values.iter().map(|v| v.abs()).sum()
    }
}

#[derive(Debug, Clone)]
pub struct Observation {
    pub from: AgentId,
    pub state: State,
    /// Rate of bearing change; values < 0.01 indicate a collision course
    pub bearing_rate: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    Hold,
    ChangeHeading(Heading),
    Refit(Component),
    Prune { target: String, reason: String },
    Broadcast(SignPattern),
}

impl fmt::Display for Action {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Action::Hold => write!(f, "Hold"),
            Action::ChangeHeading(h) => write!(f, "ChangeHeading({:?})", h.direction),
            Action::Refit(c) => write!(f, "Refit({}:{})", c.name, c.version),
            Action::Prune { target, reason } => {
                write!(f, "Prune({}: {})", target, reason)
            }
            Action::Broadcast(p) => write!(f, "Broadcast({:?})", p.0),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Heading {
    pub direction: Vec<f64>,
    pub scope: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Component {
    pub name: String,
    pub version: String,
}

/// A sign-pattern broadcast message used for inter-agent coordination
#[derive(Debug, Clone, PartialEq)]
pub struct SignPattern(pub Vec<i8>);

#[derive(Debug, Clone)]
pub struct BuildRecord {
    pub keel_date: u64,
    pub refits: Vec<String>,
    pub prunes: Vec<(String, String)>,
}

#[derive(Debug, Clone)]
pub struct LogEntry {
    pub tick: u64,
    pub phase: Phase,
    pub observation: Option<Observation>,
    pub action: Action,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Phase {
    Commissioning,
    Operational,
    Stressed,
    Recovering,
}

impl fmt::Display for Phase {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Phase::Commissioning => write!(f, "Commissioning"),
            Phase::Operational => write!(f, "Operational"),
            Phase::Stressed => write!(f, "Stressed"),
            Phase::Recovering => write!(f, "Recovering"),
        }
    }
}

// ---------------------------------------------------------------------------
// FleetAgent
// ---------------------------------------------------------------------------

pub struct FleetAgent {
    config: AgentConfig,
    state: State,
    build: BuildRecord,
    observations: Vec<Observation>,
    log: Vec<LogEntry>,
    tick: u64,
}

impl FleetAgent {
    /// Commission a new agent from config.
    pub fn new(config: AgentConfig) -> Self {
        let keel_date = config.keel_date;
        let state = State {
            values: vec![1.0], // nominal initial state
            timestamp: keel_date,
            sign_pattern: vec![1],
        };
        FleetAgent {
            config,
            state,
            build: BuildRecord {
                keel_date,
                refits: Vec::new(),
                prunes: Vec::new(),
            },
            observations: Vec::new(),
            log: Vec::new(),
            tick: 0,
        }
    }

    /// Run one full loop iteration: SENSE → CONSTRAINT CHECK → ORIENT → DECIDE → ACT → RECORD.
    /// Returns the actions decided during this tick.
    pub fn tick(&mut self, external: Vec<Observation>) -> Vec<Action> {
        // SENSE — clone external since we borrow it for sense and need to store it
        let sensed_refs: Vec<Observation> = {
            let mut out = Vec::with_capacity(external.len());
            for obs in &external {
                if obs.bearing_rate.is_finite()
                    && obs
                        .state
                        .sign_pattern
                        .iter()
                        .zip(self.state.sign_pattern.iter())
                        .any(|(a, b)| a == b)
                {
                    out.push(obs.clone());
                }
            }
            out
        };
        self.observations.extend(external);
        // CONSTRAINT CHECK
        let constraints_ok = self.check_constraints();
        // ORIENT
        let phase = self.orient();
        // DECIDE
        let actions = self.decide(&sensed_refs, constraints_ok, phase);
        // ACT (placeholder — just records)
        Self::act(&actions);
        // RECORD
        let log_obs = sensed_refs.first().cloned();
        let entry = LogEntry {
            tick: self.tick,
            phase,
            observation: log_obs,
            action: actions.first().cloned().unwrap_or(Action::Hold),
        };
        self.log.push(entry);
        self.tick += 1;
        // Evolve state slightly each tick (simple simulation)
        self.tick_state();
        actions
    }

    // -----------------------------------------------------------------------
    // SENSE
    // -----------------------------------------------------------------------

    /// Filter external observations, returning only those relevant (e.g. from
    /// known peers or observations whose sign pattern overlaps with ours).
    pub fn sense<'a>(&self, external: &'a [Observation]) -> Vec<&'a Observation> {
        external
            .iter()
            .filter(|obs| {
                // Relevance: bearing_rate is sensible OR sign_pattern overlaps
                obs.bearing_rate.is_finite()
                    && obs
                        .state
                        .sign_pattern
                        .iter()
                        .zip(self.state.sign_pattern.iter())
                        .any(|(a, b)| a == b)
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // CONSTRAINT CHECK
    // -----------------------------------------------------------------------

    /// Run all defined constraints against the current state.
    /// Returns true if ALL constraints pass.
    pub fn check_constraints(&self) -> bool {
        self.config
            .constraints
            .iter()
            .all(|c| self.eval_constraint(c))
    }

    /// Evaluate a single constraint: state values must stay below threshold.
    fn eval_constraint(&self, c: &Constraint) -> bool {
        let max_val = self.state.values.iter().cloned().fold(0.0_f64, f64::max);
        max_val < c.threshold
    }

    // -----------------------------------------------------------------------
    // ORIENT
    // -----------------------------------------------------------------------

    /// Determine the current operating phase from state and observations.
    pub fn orient(&mut self) -> Phase {
        let age = self.age();
        if age < 50 {
            return Phase::Commissioning;
        }
        let energy = self.state.energy();
        let has_stress = self.observations.iter().any(|o| o.bearing_rate < 0.01);
        if has_stress {
            if energy < 0.5 {
                // Stressed AND energy low → Recovering takes priority
                return Phase::Recovering;
            }
            return Phase::Stressed;
        }
        if energy < 0.5 {
            return Phase::Recovering;
        }
        Phase::Operational
    }

    /// Public accessor so tests / callers can read phase without mutating.
    pub fn phase(&self) -> Phase {
        let age = self.age();
        if age < 50 {
            return Phase::Commissioning;
        }
        let energy = self.state.energy();
        let has_stress = self.observations.iter().any(|o| o.bearing_rate < 0.01);
        if has_stress {
            if energy < 0.5 {
                return Phase::Recovering;
            }
            return Phase::Stressed;
        }
        if energy < 0.5 {
            return Phase::Recovering;
        }
        Phase::Operational
    }

    // -----------------------------------------------------------------------
    // DECIDE
    // -----------------------------------------------------------------------

    /// Choose actions based on phase + observations + constraint state.
    pub fn decide(
        &self,
        observations: &[Observation],
        constraints_ok: bool,
        phase: Phase,
    ) -> Vec<Action> {
        match phase {
            Phase::Commissioning => {
                // Learn, don't act
                vec![Action::Hold]
            }
            Phase::Stressed => {
                let mut actions = Vec::new();
                // Evasive heading change
                if let Some(h) = &self.config.heading {
                    let evasive = Heading {
                        direction: h.direction.iter().map(|d| -d).collect(),
                        scope: h.scope.clone(),
                    };
                    actions.push(Action::ChangeHeading(evasive));
                } else {
                    // No heading defined — prune the source of stress
                    if let Some(first) = observations.first() {
                        actions.push(Action::Prune {
                            target: first.from.clone(),
                            reason: "collision avoidance".into(),
                        });
                    }
                }
                // Warn others
                actions.push(Action::Broadcast(SignPattern(vec![-1, 1, -1])));
                actions
            }
            Phase::Recovering => {
                // Hold for stability
                vec![Action::Hold]
            }
            Phase::Operational => {
                if !constraints_ok {
                    // Violated constraints → refit
                    vec![Action::Refit(Component {
                        name: "core".into(),
                        version: format!("{}.1", self.build.refits.len()),
                    })]
                } else if observations.is_empty() {
                    vec![Action::Hold]
                } else {
                    // Respond to observations while maintaining heading
                    let mut actions = Vec::new();
                    if let Some(h) = &self.config.heading {
                        let adjusted = Heading {
                            direction: h.direction.clone(),
                            scope: h.scope.clone(),
                        };
                        actions.push(Action::ChangeHeading(adjusted));
                    }
                    actions
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // ACT
    // -----------------------------------------------------------------------

    /// Placeholder for side-effects — currently just logs actions.
    pub fn act(actions: &[Action]) {
        // In a real deployment this would talk to hardware, emit network
        // messages, etc. Here we simply trace to stderr via eprintln!.
        for a in actions {
            eprintln!("[act] executing: {}", a);
        }
    }

    // -----------------------------------------------------------------------
    // RECORD helpers
    // -----------------------------------------------------------------------

    /// Tick-level state evolution (simple decay + noise for simulation).
    fn tick_state(&mut self) {
        for v in self.state.values.iter_mut() {
            *v *= 0.99; // slight decay
            if *v < 0.01 {
                *v = 0.0;
            }
        }
        self.state.timestamp = self.config.keel_date + self.tick;
        // Simulate sign-pattern drift
        self.state.sign_pattern = self
            .state
            .sign_pattern
            .iter()
            .map(|s| {
                let r: f64 = fast_mod(self.tick as f64, 7.0) / 7.0;
                if r < 0.1 {
                    -s
                } else {
                    *s
                }
            })
            .collect();
    }

    /// Age in ticks since keel_date (but tied to tick counter).
    pub fn age(&self) -> u64 {
        self.tick
    }

    /// True if still in the first 50 ticks.
    pub fn is_commissioning(&self) -> bool {
        self.age() < 50
    }

    /// Immutable reference to the event log.
    pub fn log(&self) -> &[LogEntry] {
        &self.log
    }

    /// Immutable reference to the build record.
    pub fn build_record(&self) -> &BuildRecord {
        &self.build
    }
}

// A lightweight pseudo-random modulo helper to avoid pulling in rand.
fn fast_mod(a: f64, b: f64) -> f64 {
    (a - (a / b).floor() * b).abs()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> AgentConfig {
        AgentConfig {
            id: "test-agent".into(),
            keel_date: 1000,
            constraints: vec![Constraint {
                name: "temperature".into(),
                description: "Core temp must stay below 85.0".into(),
                threshold: 85.0,
            }],
            heading: Some(Heading {
                direction: vec![1.0, 0.0, 0.0],
                scope: vec!["north".into()],
            }),
            coupling: 0.5,
            gain: 1.0,
        }
    }

    // -----------------------------------------------------------------------
    // Construction & identity
    // -----------------------------------------------------------------------

    #[test]
    fn test_new_agent() {
        let cfg = default_config();
        let agent = FleetAgent::new(cfg.clone());
        assert_eq!(agent.config.id, "test-agent");
        assert_eq!(agent.age(), 0);
        assert!(agent.is_commissioning());
    }

    #[test]
    fn test_build_record() {
        let cfg = default_config();
        let agent = FleetAgent::new(cfg);
        assert_eq!(agent.build_record().keel_date, 1000);
        assert!(agent.build_record().refits.is_empty());
        assert!(agent.build_record().prunes.is_empty());
    }

    #[test]
    fn test_initial_state_energy() {
        let agent = FleetAgent::new(default_config());
        assert!((agent.state.energy() - 1.0).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // Age & commissioning
    // -----------------------------------------------------------------------

    #[test]
    fn test_age_increases() {
        let cfg = default_config();
        let mut agent = FleetAgent::new(cfg);
        for i in 0..10 {
            assert_eq!(agent.age(), i);
            let _ = agent.tick(vec![]);
        }
        assert_eq!(agent.age(), 10);
    }

    #[test]
    fn test_commissioning_phase_under_50() {
        let cfg = default_config();
        let mut agent = FleetAgent::new(cfg);
        for _ in 0..49 {
            let _ = agent.tick(vec![]);
        }
        assert_eq!(agent.phase(), Phase::Commissioning);
    }

    #[test]
    fn test_commissioning_ends_at_50() {
        let cfg = default_config();
        let mut agent = FleetAgent::new(cfg);
        for _ in 0..50 {
            let _ = agent.tick(vec![]);
        }
        assert_ne!(agent.phase(), Phase::Commissioning);
    }

    // -----------------------------------------------------------------------
    // Phase transitions
    // -----------------------------------------------------------------------

    #[test]
    fn test_operational_after_commissioning() {
        let cfg = default_config();
        let mut agent = FleetAgent::new(cfg);
        for _ in 0..60 {
            let _ = agent.tick(vec![]);
        }
        assert_eq!(agent.phase(), Phase::Operational);
    }

    #[test]
    fn test_stressed_phase_on_low_bearing_rate() {
        let cfg = default_config();
        let mut agent = FleetAgent::new(cfg);
        for _ in 0..55 {
            let _ = agent.tick(vec![]);
        }
        let inbound = vec![Observation {
            from: "hostile".into(),
            state: State {
                values: vec![10.0],
                timestamp: 0,
                sign_pattern: vec![1],
            },
            bearing_rate: 0.005,
        }];
        // Feed the observation into tick, then check phase
        let _ = agent.tick(inbound);
        assert_eq!(agent.phase(), Phase::Stressed);
    }

    #[test]
    fn test_recovering_on_low_energy() {
        let cfg = default_config();
        let mut agent = FleetAgent::new(cfg);
        for _ in 0..55 {
            let _ = agent.tick(vec![]);
        }
        // Force energy low by setting state directly
        agent.state.values = vec![0.1];
        assert!(agent.state.energy() < 0.5);
        assert_eq!(agent.phase(), Phase::Recovering);
    }

    #[test]
    fn test_recovering_takes_priority_over_stressed() {
        let cfg = default_config();
        let mut agent = FleetAgent::new(cfg);
        for _ in 0..55 {
            let _ = agent.tick(vec![]);
        }
        agent.state.values = vec![0.1];
        let inbound = vec![Observation {
            from: "hostile".into(),
            state: State {
                values: vec![10.0],
                timestamp: 0,
                sign_pattern: vec![1],
            },
            bearing_rate: 0.001,
        }];
        let _ = agent.tick(inbound);
        // Low energy + stress → recovering
        assert_eq!(agent.phase(), Phase::Recovering);
    }

    // -----------------------------------------------------------------------
    // SENSE
    // -----------------------------------------------------------------------

    #[test]
    fn test_sense_relevant_observations() {
        let cfg = default_config();
        let agent = FleetAgent::new(cfg);
        let relevant = Observation {
            from: "friend".into(),
            state: State {
                values: vec![2.0],
                timestamp: 0,
                sign_pattern: vec![1],
            },
            bearing_rate: 0.5,
        };
        let irrelevant_sig = Observation {
            from: "stranger".into(),
            state: State {
                values: vec![2.0],
                timestamp: 0,
                sign_pattern: vec![-1],
            },
            bearing_rate: 0.5,
        };
        let nonsense = Observation {
            from: "noise".into(),
            state: State {
                values: vec![0.0],
                timestamp: 0,
                sign_pattern: vec![],
            },
            bearing_rate: f64::NAN,
        };
        let obs = vec![relevant, irrelevant_sig, nonsense];
        let sensed = agent.sense(&obs);
        assert_eq!(sensed.len(), 1);
        assert_eq!(sensed[0].from, "friend");
    }

    #[test]
    fn test_sense_empty_when_no_overlap() {
        let cfg = default_config();
        let agent = FleetAgent::new(cfg);
        let obs = vec![Observation {
            from: "alien".into(),
            state: State {
                values: vec![1.0],
                timestamp: 0,
                sign_pattern: vec![-1],
            },
            bearing_rate: 3.0,
        }];
        assert!(agent.sense(&obs).is_empty());
    }

    // -----------------------------------------------------------------------
    // CONSTRAINT CHECK
    // -----------------------------------------------------------------------

    #[test]
    fn test_constraint_passes() {
        let cfg = default_config();
        let agent = FleetAgent::new(cfg);
        assert!(agent.check_constraints()); // initial state = 1.0 < 85.0
    }

    #[test]
    fn test_constraint_fails() {
        let mut cfg = default_config();
        cfg.constraints = vec![Constraint {
            name: "tight".into(),
            description: "Strict".into(),
            threshold: 0.5,
        }];
        let agent = FleetAgent::new(cfg);
        assert!(!agent.check_constraints());
    }

    #[test]
    fn test_constraint_uses_max_value() {
        let mut cfg = default_config();
        cfg.constraints = vec![Constraint {
            name: "multi".into(),
            description: "Multi-dim".into(),
            threshold: 3.0,
        }];
        let mut agent = FleetAgent::new(cfg);
        agent.state.values = vec![0.5, 2.0, 0.1];
        assert!(agent.check_constraints());
        agent.state.values = vec![0.5, 5.0, 0.1];
        assert!(!agent.check_constraints());
    }

    // -----------------------------------------------------------------------
    // DECIDE
    // -----------------------------------------------------------------------

    #[test]
    fn test_decide_commissioning_hold() {
        let cfg = default_config();
        let agent = FleetAgent::new(cfg);
        let actions = agent.decide(&[], true, Phase::Commissioning);
        assert_eq!(actions, vec![Action::Hold]);
    }

    #[test]
    fn test_decide_stressed_evasive() {
        let cfg = default_config();
        let agent = FleetAgent::new(cfg);
        let obs = vec![Observation {
            from: "threat".into(),
            state: State {
                values: vec![5.0],
                timestamp: 0,
                sign_pattern: vec![1],
            },
            bearing_rate: 0.001,
        }];
        let actions = agent.decide(&obs, true, Phase::Stressed);
        assert_eq!(actions.len(), 2);
        assert!(matches!(actions[0], Action::ChangeHeading(_)));
        assert!(matches!(actions[1], Action::Broadcast(_)));
    }

    #[test]
    fn test_decide_stressed_no_heading_prune() {
        let mut cfg = default_config();
        cfg.heading = None;
        let agent = FleetAgent::new(cfg);
        let obs = vec![Observation {
            from: "rogue".into(),
            state: State {
                values: vec![99.0],
                timestamp: 0,
                sign_pattern: vec![1],
            },
            bearing_rate: 0.0001,
        }];
        let actions = agent.decide(&obs, true, Phase::Stressed);
        assert_eq!(actions.len(), 2);
        assert!(matches!(&actions[0], Action::Prune { target, .. } if target == "rogue"));
        assert!(matches!(actions[1], Action::Broadcast(_)));
    }

    #[test]
    fn test_decide_recovering_hold() {
        let cfg = default_config();
        let agent = FleetAgent::new(cfg);
        let actions = agent.decide(&[], true, Phase::Recovering);
        assert_eq!(actions, vec![Action::Hold]);
    }

    #[test]
    fn test_decide_operational_constraint_violation_refit() {
        let cfg = default_config();
        let agent = FleetAgent::new(cfg);
        let actions = agent.decide(&[], false, Phase::Operational);
        assert_eq!(actions.len(), 1);
        assert!(matches!(&actions[0], Action::Refit(c) if c.name == "core"));
    }

    #[test]
    fn test_decide_operational_no_observations_hold() {
        let cfg = default_config();
        let agent = FleetAgent::new(cfg);
        let actions = agent.decide(&[], true, Phase::Operational);
        assert_eq!(actions, vec![Action::Hold]);
    }

    #[test]
    fn test_decide_operational_with_observations() {
        let cfg = default_config();
        let agent = FleetAgent::new(cfg);
        let obs = vec![Observation {
            from: "peer".into(),
            state: State {
                values: vec![1.0],
                timestamp: 0,
                sign_pattern: vec![1],
            },
            bearing_rate: 1.0,
        }];
        let actions = agent.decide(&obs, true, Phase::Operational);
        // Response with heading adjustment
        assert!(matches!(actions[0], Action::ChangeHeading(_)));
    }

    // -----------------------------------------------------------------------
    // Full tick integration
    // -----------------------------------------------------------------------

    #[test]
    fn test_tick_increments_age_and_logs() {
        let cfg = default_config();
        let mut agent = FleetAgent::new(cfg);
        assert_eq!(agent.log.len(), 0);
        let actions = agent.tick(vec![]);
        assert_eq!(agent.age(), 1);
        assert_eq!(agent.log.len(), 1);
        assert_eq!(agent.log[0].tick, 0);
        assert_eq!(agent.log[0].phase, Phase::Commissioning);
        assert_eq!(actions, vec![Action::Hold]);
    }

    #[test]
    fn test_one_hundred_ticks_no_panic() {
        let cfg = default_config();
        let mut agent = FleetAgent::new(cfg);
        for _ in 0..100 {
            let external = vec![
                Observation {
                    from: "drone-1".into(),
                    state: State {
                        values: vec![0.8],
                        timestamp: 0,
                        sign_pattern: vec![1],
                    },
                    bearing_rate: rand_bearing(),
                },
                Observation {
                    from: "drone-2".into(),
                    state: State {
                        values: vec![1.2],
                        timestamp: 0,
                        sign_pattern: vec![1],
                    },
                    bearing_rate: rand_bearing(),
                },
            ];
            agent.tick(external);
        }
        assert_eq!(agent.age(), 100);
        assert_eq!(agent.log.len(), 100);
        // Should have transitioned through phases
        let phases: Vec<Phase> = agent.log.iter().map(|e| e.phase).collect();
        assert!(phases.contains(&Phase::Commissioning));
        assert!(phases.contains(&Phase::Operational));
    }

    #[test]
    fn test_stress_event_during_run() {
        let cfg = default_config();
        let mut agent = FleetAgent::new(cfg);
        // Run past commissioning
        for _ in 0..55 {
            let _ = agent.tick(vec![]);
        }
        let obs = vec![Observation {
            from: "missile".into(),
            state: State {
                values: vec![100.0],
                timestamp: 0,
                sign_pattern: vec![1],
            },
            bearing_rate: 0.0001,
        }];
        let actions = agent.tick(obs);
        // Should produce stress actions
        let has_evasive = actions
            .iter()
            .any(|a| matches!(a, Action::ChangeHeading(_)));
        let has_broadcast = actions.iter().any(|a| matches!(a, Action::Broadcast(_)));
        assert!(
            has_evasive || has_broadcast,
            "Expected stress-related actions"
        );
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_zero_constraints() {
        let cfg = AgentConfig {
            id: "zero".into(),
            keel_date: 0,
            constraints: vec![],
            heading: None,
            coupling: 0.0,
            gain: 0.0,
        };
        let agent = FleetAgent::new(cfg);
        assert!(agent.check_constraints()); // vacuously true
    }

    #[test]
    fn test_empty_state_values() {
        let cfg = default_config();
        let mut agent = FleetAgent::new(cfg);
        agent.state.values = vec![];
        assert_eq!(agent.state.energy(), 0.0);
        // Should still function
        let _ = agent.tick(vec![]);
    }

    #[test]
    fn test_fast_mod_handles_negative() {
        let r = fast_mod(-3.0, 7.0);
        assert!((r - 4.0).abs() < 1e-9); // -3 mod 7 = 4
    }

    #[test]
    fn test_fast_mod_zero() {
        let r = fast_mod(0.0, 5.0);
        assert!((r - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_action_display() {
        assert_eq!(format!("{}", Action::Hold), "Hold");
        let h = Action::ChangeHeading(Heading {
            direction: vec![1.0, 0.0],
            scope: vec!["north".into()],
        });
        let s = format!("{}", h);
        assert!(s.contains("ChangeHeading"));
    }

    #[test]
    fn test_phase_display() {
        assert_eq!(format!("{}", Phase::Commissioning), "Commissioning");
        assert_eq!(format!("{}", Phase::Operational), "Operational");
        assert_eq!(format!("{}", Phase::Stressed), "Stressed");
        assert_eq!(format!("{}", Phase::Recovering), "Recovering");
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /// Produce a "random" bearing rate without depending on rand crate.
    fn rand_bearing() -> f64 {
        // Simple deterministic value for testing
        0.5 + (fast_mod(42.0, 13.0) / 13.0)
    }
}
