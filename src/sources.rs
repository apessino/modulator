//!
//! # Modulator Sources
//!
//! A collection of types that implement the Modulator trait
//!
//! Copyright© 2018 Ready At Dawn Studios

use rand::prelude::*;

use std::f32;
use std::any::Any;

use crate::Modulator;
use crate::ModulatorEnv;

///
/// Simple modulator using a value closure/`Fn`, with frequency and amplitude. The
/// closure receives self, elapsed time (in seconds) and returns a new value.
//
pub struct Wave {
    pub amplitude: f32,
    pub frequency: f32,

    pub wave: Box<dyn Fn(&Wave, f32) -> f32>, // wave closure, receives self and time in s

    pub time: u64,  // accumulated microseconds
    pub value: f32, // current value

    pub enabled: bool, // enabling toggle
}

impl Wave {
    /// Build a sin wave with given amplitude and frequency
    pub fn new(amplitude: f32, frequency: f32) -> Self {
        Self {
            amplitude,
            frequency,

            wave: Box::new(|_, _| 0.0),

            time: 0,
            value: 0.0,

            enabled: true,
        }
    }

    /// Builder: set the wave calculation closure
    pub fn wave(mut self, wave: Box<dyn Fn(&Wave, f32) -> f32>) -> Self {
        self.wave = wave;
        self
    }

    /// Builder: set the wave calculation closure
    pub fn set_wave(&mut self, wave: Box<dyn Fn(&Wave, f32) -> f32>) {
        self.wave = wave;
    }
}

impl Modulator<f32> for Wave {
    fn value(&self) -> f32 {
        self.value
    }
    fn range(&self) -> Option<[f32; 2]> {
        Some([-self.amplitude, self.amplitude])
    }
    fn goal(&self) -> Option<f32> {
        Some(self.value)
    }

    fn elapsed_us(&self) -> u64 {
        self.time
    }
    fn enabled(&self) -> bool {
        self.enabled
    }
    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    fn advance(&mut self, dt: u64) {
        self.time += dt;
        self.value = (self.wave)(self, ModulatorEnv::<f32>::micros_to_secs(self.time));
    }

    fn as_any(&mut self) -> &mut dyn Any {
        self
    }
}

///
/// Critically damped spring modulator. Moves towards its set `goal` with `smooth` seconds
/// of delay, critically damping its arrival so it slows down and stops at the goal without
/// overshooting or oscillation.
///
/// If overshooting is desired, positive values of `undamp` can be set to add artificial
/// overshoot/oscillations around the goal.
///

pub struct ScalarSpring {
    pub smooth: f32, // spring delay (smoothing), in seconds
    pub undamp: f32, // artificial undamping (adds overshoot)

    pub goal: f32,  // current target for the spring
    pub value: f32, // current position of the spring (value)

    pub vel: f32,  // current velocity
    pub time: u64, // accumulated microseconds

    pub enabled: bool, // enabling toggle
}

impl ScalarSpring {
    /// Make a critically damped spring
    pub fn new(smooth: f32, undamp: f32, initial: f32) -> Self {
        Self {
            smooth,
            undamp,

            goal: initial,
            value: initial,

            vel: 0.0,
            time: 0,

            enabled: true,
        }
    }

    /// Update the target the spring is moving towards
    pub fn spring_to(&mut self, goal: f32) {
        self.goal = goal;
    }

    /// Jump immediately to the given goal, zero velocity
    pub fn jump_to(&mut self, goal: f32) {
        self.goal = goal;
        self.value = goal;
        self.vel = 0.0;
    }
}

impl Modulator<f32> for ScalarSpring {
    fn value(&self) -> f32 {
        self.value
    }
    fn goal(&self) -> Option<f32> {
        Some(self.goal)
    }
    fn set_goal(&mut self, goal: f32) {
        self.spring_to(goal);
    }

    fn elapsed_us(&self) -> u64 {
        self.time
    }
    fn enabled(&self) -> bool {
        self.enabled
    }
    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    fn advance(&mut self, dt: u64) {
        self.time += dt;

        if self.smooth < 0.0001 {
            self.value = self.goal; // not enough time, spring too strong
            self.vel = 0.0;
        } else {
            let dt = ModulatorEnv::<f32>::micros_to_secs(dt);

            let omega = 2.0 / self.smooth;
            let x = omega * dt;
            let ex = 1.0 / x.exp();
            let ud = dt * self.undamp;

            let d = self.value - self.goal;
            let t = (self.vel + omega * d) * dt;
            let v = self.vel;

            self.vel = (v - omega * t) * ex + v * ud;
            self.value = self.goal + (d + t) * ex;
        }
    }

    fn as_any(&mut self) -> &mut dyn Any {
        self
    }
}

///
/// A programmable goal follower. Picks a `goal` within one of its `regions` for its owned
/// `follower` modulator, then  monitor its progress until the follower gets to `threshold`
/// distance to the goal and has velocity of `vel_threshold` or less, at which point it
/// considers it arrived.
///
/// Once a goal has been reached, it picks a pause duration microseconds from `pause_range`,
/// waits for the pause to elapse, then picks a new goal and repeats the process.
///
/// This modulator can be given any other modulator type as its owned `follower`, but a
/// type that is unable to pursue and arrive to its given `goal` is, of course, never going
/// to satisfy the conditions for arrival.
///

pub struct ScalarGoalFollower {
    pub regions: Vec<[f32; 2]>, // set of regions to pick goals from
    pub random_region: bool,    //  cycle regions randomly instead of sequentially

    pub threshold: f32, // threshold to consider a goal reached and move on to next one
    pub vel_threshold: f32, // as above, but for velocity

    pub pause_range: [u64; 2], // range of pause microseconds to pick between goals

    pub follower: Box<dyn Modulator<f32>>, // the modulator that follows the current goal

    pub current_region: usize, // index of last region used to set the goal
    pub paused_left: u64,      // when set, number of microseconds until the pause ends
    pub time: u64,             // accumulated microseconds

    pub enabled: bool, // enabling toggle
}

impl ScalarGoalFollower {
    /// Make a scalar goal follower
    pub fn new(follower: Box<dyn Modulator<f32>>) -> Self {
        Self {
            regions: vec![],
            random_region: false,

            threshold: 0.01,
            vel_threshold: 0.0001,

            pause_range: [0, 0],

            follower,

            current_region: 0,
            paused_left: 0,
            time: 0,

            enabled: true,
        }
    }

    /// Select and begin following a new goal
    fn set_new_goal(&mut self) {
        let n = self.regions.len(); // current number of regions, if 0 we do nothing here
        if n > 0 {
            self.current_region = if self.random_region {
                thread_rng().gen_range(0, n)
            } else if self.current_region + 1 < n {
                self.current_region + 1
            } else {
                0
            };

            let region = &self.regions[self.current_region]; // region we are going to
            let goal = if region[1] > region[0] {
                thread_rng().gen_range(region[0], region[1])
            } else {
                region[0]
            };

            self.follower.set_goal(goal); // new goal
        }
    }
}

impl Modulator<f32> for ScalarGoalFollower {
    fn value(&self) -> f32 {
        self.follower.value()
    }
    fn range(&self) -> Option<[f32; 2]> {
        let mut r = if !self.regions.is_empty() {
            self.regions[0]
        } else {
            [0.0, 0.0]
        };

        for j in self.regions.iter().skip(1) {
            // union of all active ranges
            if j[0] < r[0] {
                r[0] = j[0];
            }
            if j[1] > r[1] {
                r[1] = j[1];
            }
        }
        Some(r)
    }

    fn goal(&self) -> Option<f32> {
        // these just forward to the follower
        self.follower.goal()
    }
    fn set_goal(&mut self, goal: f32) {
        self.follower.set_goal(goal);
    }

    fn elapsed_us(&self) -> u64 {
        self.time
    }

    fn enabled(&self) -> bool {
        self.enabled
    }
    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    fn advance(&mut self, dt: u64) {
        self.time += dt;

        if self.paused_left > 0 {
            self.paused_left -= u64::min(self.paused_left, dt); // elapse paused state
        } else {
            let p0 = self.follower.value();
            self.follower.advance(dt);
            let p1 = self.follower.value();

            let secs = ModulatorEnv::<f32>::micros_to_secs(dt);
            let vel = if secs > f32::MIN {
                (p1 - p0) / secs
            } else {
                0.0
            };

            if (p1 - self.follower.goal().unwrap()).abs() > self.threshold
                || vel.abs() > self.vel_threshold
            {
                return; // still moving towards the goal
            }

            self.paused_left = if self.pause_range[1] > self.pause_range[0] {
                thread_rng().gen_range(self.pause_range[0], self.pause_range[1])
            } else {
                self.pause_range[0]
            };
        }

        if self.paused_left == 0 {
            self.set_new_goal(); // done pausing, resume following
        }
    }

    fn as_any(&mut self) -> &mut dyn Any {
        self
    }
}

///
/// A modulator that uses classical mechanics to move to its `goal` - it guarantees smooth
/// acceleration, deceleration and speed limiting regardless of settings.
///
/// The goal calculation computes an analytical solution to the motion equation. When
/// a new goal is set, `speed_limit`, `acceleration` and `deceleration` values are
/// picked from their respective ranges, then movement begins with the value starting
/// from current value with 0 velocity, accelerating at the selected rate up to the speed
/// limit, then decelerating at the selected rate of deceleration so that it is _guaranteed_
/// to come to a stop at the goal.
///
/// The analytical solution to the motion equation ensures that, regardless of input, the
/// value always accelerates and decelerates at the picked rates, and never exceeds the
/// speed max. If there is not enough time to reach peak speed, the value accelerates as
/// much as it it can while ensuring that it will decelerate and come to a stop (0 speed)
/// exactly at `goal`.
///

pub struct Newtonian {
    pub speed_limit: [f32; 2], // max speed range, selected on new goal

    pub acceleration: [f32; 2], // range of acceleration values selected on new goal
    pub deceleration: [f32; 2], // range of deceleration values selected on new goal

    pub goal: f32,  // current goal
    pub value: f32, // current position of the particle (value)

    pub time: u64,     // accumulated microseconds since the most recent goal was set
    pub enabled: bool, // enabling toggle

    s: f32,      // speed selected for the current goal
    a: f32,      // accel selected towards the goal
    d: f32,      // decel selected towards the goal
    f: f32,      // location we started moving from
    t: [f32; 3], // phase times: t[0] accel, t[1] sustain, t[2] decel
}

impl Newtonian {
    /// Make a Newtonian mechanics value moving modulator
    pub fn new(
        speed_limit: [f32; 2],
        acceleration: [f32; 2],
        deceleration: [f32; 2],
        initial: f32,
    ) -> Self {
        Self {
            speed_limit,

            acceleration,
            deceleration,

            goal: initial,
            value: initial,

            time: 0,
            enabled: true,

            s: 0.0,
            a: 0.0,
            d: 0.0,
            f: initial,
            t: [0.0; 3],
        }
    }

    /// Manual reset of the current value - also ends all motion
    pub fn reset(&mut self, value: f32) {
        self.value = value;
        self.goal = value;

        self.s = 0.0;
        self.a = 0.0;
        self.d = 0.0;
        self.f = value;
        self.t = [0.0; 3];
    }

    /// Start moving from the current value to the given goal
    pub fn move_to(&mut self, goal: f32) {
        self.time = 0; // for this modulator time is elapsed us in goal
        self.goal = goal;

        self.s = Newtonian::gen_value(&self.speed_limit);
        self.a = Newtonian::gen_value(&self.acceleration);
        self.d = Newtonian::gen_value(&self.deceleration);
        self.f = self.value;

        self.calculate_events();
    }

    /// Calculate the event times for current settings - set corrected internal state
    /// as well as the timeline of the trip to goal in t[]
    fn calculate_events(&mut self) {
        let x = (self.goal - self.f).abs();

        let a = if self.a > f32::EPSILON {
            self.a
        } else {
            1_000_000.0
        };
        let d = if self.d > f32::EPSILON {
            self.d
        } else {
            1_000_000.0
        };
        let r = a / d;

        self.t[0] = (x * 2.0 / (a * (1.0 + r))).sqrt();

        let mut v = a * self.t[0];
        if v > self.s {
            v = self.s;
            self.t[0] = self.s / a; // back up to when we hit the limit
        } else {
            self.s = v; // will be needed to back off from overshot decel range
        }

        self.t[2] = self.t[0] * r; // ratio converts one accel into decel

        let d0 = self.t[0] * self.t[0] * a * 0.5;
        let d2 = self.t[2] * self.t[2] * d * 0.5;

        self.t[1] = (x - d0 - d2) / v; // time spent at max speed

        if self.goal > self.f {
            self.a = a;
            self.d = -d;
        } else {
            self.s = -self.s;
            self.a = -a;
            self.d = d;
        }

        self.t[1] = self.t[0] + self.t[1]; // make the durations into a timeline
        self.t[2] = self.t[1] + self.t[2];
    }

    /// Displacement due to constant acceleration `a` over `t` seconds
    fn accelerate(a: f32, t: f32) -> f32 {
        a * t * t * 0.5
    }
    /// Displacement due to constant speed `s` over `t` seconds
    fn forward(s: f32, t: f32) -> f32 {
        s * t
    }

    /// Get a value in the given range (need the test since gen_range panics on a null range)
    fn gen_value(r: &[f32]) -> f32 {
        if r[1] > r[0] {
            thread_rng().gen_range(r[0], r[1])
        } else {
            r[0]
        }
    }
}

impl Modulator<f32> for Newtonian {
    fn value(&self) -> f32 {
        self.value
    }

    fn elapsed_us(&self) -> u64 {
        self.time
    }
    fn enabled(&self) -> bool {
        self.enabled
    }
    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    fn advance(&mut self, dt: u64) {
        self.time += dt;

        let t = ModulatorEnv::<f32>::micros_to_secs(self.time); // time to goal
        let (t0, t1, t2) = (self.t[0], self.t[1], self.t[2]);

        self.value = self.f + Newtonian::accelerate(self.a, f32::min(t, t0));
        if t > t0 {
            self.value = self.value + Newtonian::forward(self.s, f32::min(t, t2) - t0);
            if t > t1 {
                self.value = self.value + Newtonian::accelerate(self.d, f32::min(t, t2) - t1);
            }
        }
    }

    fn as_any(&mut self) -> &mut dyn Any {
        self
    }
}

///
/// Inspired by classic analog shift registers like those used in Buchla synthesizers, this
/// modulator has a vector of values `buckets` containing values selected from `value_range`.
///
/// A `period` of the register is the length of time, in seconds, that the value takes to
/// visit all the buckets in the register. Once a period is over, the value moves back to
/// the first bucket and continues to move.
///
/// If `interp` is `ShiftRegisterInterp::None` then the value returned corresponds to the
/// current bucket being visited. If it is `ShiftRegisterInterp::Linear` then it is the
/// linear interpolation of the current bucket and the next. If it is
/// `ShiftRegisterInterp::Quadratic` then the value is the result of polynomial interpolation
/// of the values of the previous, current and next bucket.
///
/// Every time the value leaves a bucket (it is done visiting it for the period) it has
/// `odds` chances of _replacing_ the value in the bucket it just left, where `odds` ranges
/// from 0.0 (value never changes) to 1.0 (value always changes).
///
/// Parameter `age_range` can be used to specify an age (in periods) over which the odds
/// of a value changing increase linearly. For example: if `odds` is set to 0.1 (10%) and
/// `age_range` is set to [200, 1000) then for the first 200 periods a value's odds of
/// changing are 10%, and between 200 and 1000 periods they increase from 10% to 100%. By
/// default `age_range` is set to [u32::MAX, u32::MAX] so the odds never change.
///
/// The result is that the shift register is periodic and exhibits a pattern (given low
/// enough odds), but still evolves over time in an organic way.
///

pub struct ShiftRegister {
    pub buckets: Vec<f32>, // current array of values
    pub ages: Vec<u32>,    // symmetrical array of value ages (number of periods since updating)

    pub value_range: [f32; 2], // range of values selected when picking a bucket value [..)
    pub odds: f32, // odds of changing a bucket's value as it is left, 0..1 (0 = never, 1 = always)
    pub age_range: [u32; 2], // min/max range over which the odds increase to 100%

    pub period: f32,                 // duration of a register loop, in seconds
    pub interp: ShiftRegisterInterp, // interpolation type

    pub time: u64,  // accumulated microseconds, local time within the register loop
    pub value: f32, // current value

    pub enabled: bool, // enabling toggle
}

/// Available choices of interpolation for shift registers
pub enum ShiftRegisterInterp {
    Linear,    // linearly interpolate between samples
    Quadratic, // quadratic interpolation between the samples
    None,      // no interpolation, buckets are read directly
}

impl ShiftRegister {
    /// Build a new shift register modulator
    pub fn new(
        buckets: usize,
        value_range: [f32; 2],
        odds: f32,
        period: f32,
        interp: ShiftRegisterInterp,
    ) -> Self {
        let b = Self::new_buckets(buckets, &value_range);
        let v = if buckets > 0 { b[0] } else { 0.0 };

        Self {
            buckets: b,
            ages: vec![0; buckets],

            value_range,
            odds,
            age_range: [std::u32::MAX; 2],

            period,
            interp,

            time: 0,
            value: v,

            enabled: true,
        }
    }

    /// Make a new bucket vector of `buckets` capacity, fill it with values from `range`
    fn new_buckets(buckets: usize, range: &[f32; 2]) -> Vec<f32> {
        let mut b = Vec::with_capacity(buckets);
        for _ in 0..buckets {
            b.push(thread_rng().gen_range(range[0], range[1]));
        }
        b // moves it out
    }

    /// Total time of a shift register loop (period) in microseconds
    fn total_period(&self) -> u64 {
        (self.period * 1_000_000.0) as u64
    }

    /// Time spent visiting a bucket, in microseconds
    ///
    /// # Panics
    ///
    /// Panics if the register's buckets vector is empty
    fn bucket_period(&self) -> u64 {
        let n = self.buckets.len();
        assert!(n > 0);

        self.total_period() / n as u64
    }

    /// Return the bucket index after the one we are given
    fn next_bucket(&self, i: usize, n: usize) -> usize {
        debug_assert!(n > 0);
        if i < n - 1 {
            i + 1
        } else {
            0
        }
    }

    /// Return the bucket index before the one we are given
    fn previous_bucket(&self, i: usize, n: usize) -> usize {
        debug_assert!(n > 0);
        if i > 0 {
            i - 1
        } else {
            n - 1
        }
    }
}

impl Modulator<f32> for ShiftRegister {
    fn value(&self) -> f32 {
        self.value
    }
    fn range(&self) -> Option<[f32; 2]> {
        Some(self.value_range)
    }

    fn elapsed_us(&self) -> u64 {
        self.time
    }

    fn enabled(&self) -> bool {
        self.enabled
    }
    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    fn advance(&mut self, dt: u64) {
        let n = self.buckets.len(); // number of buckets in vector
        let p = self.total_period(); // microseconds length of one register loop
        let bp = self.bucket_period(); // microseconds spent visiting each bucket

        if n == 0 || p == 0 || bp == 0 {
            return;
        }

        let pt = self.time % p; // convert accumulated time into period time
        let mut bi = usize::min((pt / bp) as usize, n - 1); // current bucket in period

        let bt = pt - bp * bi as u64; // time already spent visiting the current bucket
        let r = (bt + dt) / bp; // number of buckets we are going to visit

        for _ in 0..r {
            let bh = self.previous_bucket(bi, n); // one behind so we don't affect quadratic
            let mut odds = f32::min(f32::max(0.0, self.odds), 1.0);

            if self.ages[bh] >= self.age_range[0] && self.age_range[0] < self.age_range[1] {
                let t = f32::min(
                    (self.ages[bh] - self.age_range[0]) as f32
                        / (self.age_range[1] - self.age_range[0]) as f32,
                    1.0,
                );
                odds = odds + (1.0 - odds) * t;
            }

            if thread_rng().gen_range(0.0, 1.0) < odds {
                self.buckets[bh] = thread_rng().gen_range(self.value_range[0], self.value_range[1]);
                self.ages[bh] = 0;
            } else {
                self.ages[bh] += 1; // another period without changing value
            }

            bi = self.next_bucket(bi, n);
        }

        self.time += dt;

        match self.interp {
            ShiftRegisterInterp::Quadratic => {
                let bh = self.previous_bucket(bi, n);
                let bj = self.next_bucket(bi, n);

                let v1 = self.buckets[bi];
                let v0 = (self.buckets[bh] + v1) * 0.5;
                let v2 = (self.buckets[bj] + v1) * 0.5;

                let bt = self.time % p - bp * bi as u64;
                let tt = bt as f32 / bp as f32;

                let a0 = v0 + (v1 - v0) * tt;
                let a1 = v1 + (v2 - v1) * tt;

                self.value = a0 + (a1 - a0) * tt;
            }

            ShiftRegisterInterp::Linear => {
                let v0 = self.buckets[bi];
                let v1 = self.buckets[self.next_bucket(bi, n)];

                let bt = self.time % p - bp * bi as u64;
                self.value = v0 + (v1 - v0) * (bt as f32 / bp as f32);
            }

            ShiftRegisterInterp::None => self.value = self.buckets[bi],
        }
    }

    fn as_any(&mut self) -> &mut dyn Any {
        self
    }
}
