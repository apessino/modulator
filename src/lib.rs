#![deny(missing_docs)]

//!
//! # Modulator
//!
//! <a href="http://www.youtube.com/watch?feature=player_embedded&v=n-txrCMvdms"
//! target="_blank"><img src="http://img.youtube.com/vi/n-txrCMvdms/0.jpg"
//! alt="CLICK HERE for a Video Introduction To The Modulator Crate And Play Application"
//! width="240" height="180" border="2" /></a>
//!
//!
//! [CLICK HERE to go to the Modulator Play application repository](https://github.com/apessino/modulator_play)
//!
//! [CLICK HERE to go to the Modulator crate repository](https://github.com/apessino/modulator)
//!
//! A trait for abstracted, decoupled modulation sources. This crate includes:
//!
//!   1. The `Modulator<T>` trait definition
//!   2. An environment (host) type for modulators `ModulatorEnv<T>`
//!   3. A number of ready to use types that implement the modulator trait
//!
//! **Introduction**
//! -----
//!
//! Modulators are _sources_ of change over time which exist independently of the
//! parameters they affect, their _destinations_.
//!
//! The architecture presented here was inspired, in part, by the world of _audio
//! synthesis_, so let us introduce the main concepts by drawing a parallel to it.
//!
//! A synthesizer is a musical instrument in which electronic waveform generators
//! produce a basic sound, which is then filtered, amplified and output.
//! While the method of waveform generation and the processing applied to it are
//! key to the sonic result, by themselves they are not sufficient to produce
//! interesting, lively results.
//!
//! To increase complexity and depth, _modulations_ can be added to mutate synthesis
//! parameters over time and produce evolving, organic sounds. A synthesist
//! can sculpt the output by _connecting_ modulation sources to destinations.
//!
//! Modulation sources include periodic functions, low frequency oscillators, noise
//! generators, performance controls, etc.
//!
//! Destinations are typically parameters that affect the amplitude, frequency, or
//! harmonic structure of the sound. This results in effects such as tremolo, vibrato,
//! spectral and timbric variations to waveforms over time.
//!
//! Modulation sources and destinations should ideally be completely decoupled. A
//! destination should be able to factor input from a compatible source by establishing
//! a connection between the two.
//!
//! This generic approach, which originated with modular synthesizers, adds enormous
//! breadth to the range of sounds that can be programmed for an instrument.
//!
//! The same modulation model that makes these electronic sounds rich can be used in other
//! domains. Modulations can add life and variety to any set of parameters used by a computer
//! program. Non-interactive visual elements can be animated, user feedback can be augmented,
//! AI entity behavior can evolve over time, and much more.
//!
//! **Useful modulators included**
//! -----
//!
//! When it comes to animating an attribute, be it visual, auditory or behavioral, it is
//! often the case that we want the result to be:
//!
//! * Random, unscripted and without a scripted _feel_
//! * Controllable, precisely bound
//! * Dependably smooth, with no singularities
//! * Physically correct, instinctively pleasing
//!
//! This crate provides modulators such as `ScalarSpring`, `Newtonian`, `ScalarGoalFollower`
//! and `ShiftRegister` which, by themselves and in combination, allow the creation
//! of modulations that have some or all of the properties above.
//!
//! **How modulators work**
//! -----
//!
//! A modulator needs to be able to do at least the following:
//!
//! * Return its value at the current moment in time
//! * Evolve its status as a function of advancing time
//!
//! Let `m` be a value of a type that implements the `Modulator` trait, then:
//!
//! ```ignore
//! let value = m.value();
//! ```
//!
//! returns the current value of the modulator. To evolve the modulator by `dt`
//! microseconds use:
//!
//! ```ignore
//! m.advance(dt);
//! ```
//!
//! In practice, the latter is rarely done directly, as using an _environment_ (a host
//! for modulators) such as the included `ModulatorEnv` type, is much more convenient.
//!
//! **Modulator environments**
//! -----
//!
//! The `ModulatorEnv<T>` type is an _owning host_ for modulators. Generally, you create
//! one or more environments in your application, such as:
//!
//! ```ignore
//! // Somewhere in a struct...
//! m1: ModulatorEnv<f32>, // hosts modulators that give scalar f32 values
//!
//! // Somewhere in constructor of that struct...
//! m1: ModulatorEnv::new(),
//! ```
//!
//! The above creates a modulator environment `m1` in a struct, probably a modulation
//! struct that collects all state/data related to modulation for the app.
//!
//! Then, somewhere in the application, the environment must be _ticked_ forward by the
//! elapsed `dt` microseconds of the current frame, like this:
//!
//! ```ignore
//! // Here st is the modulation data struct that contains m1, dt is elapsed micros
//! st.m1.advance(dt);
//! ```
//!
//! The environment advances all the enabled modulators it hosts. It is important
//! to notice two things about `ModulatorEnv`:
//!
//!   1. The environment owns the modulators it hosts
//!   2. The environment is generic in the same value T as its hosted modulators
//!
//! Point 2 means that, since trait `Modulator<T>` is generic in T, the value type,
//! then all modulators in an environment must have the same T. All modulator types
//! provided with this crate are `Modulator<f32>`, that is: their value is a scalar
//! of type `f32`.
//!
//! Point 1 means that the lifetime of the modulator is managed by the environment,
//! so you can "create and give" your modulators and let the environment drop them
//! when it is dropped (`ModulatorEnv` provides methods to manually manage the lifetime
//! of its modulators, if desired).
//!
//! Here is an example using the `Wave` modulator. The `Wave` modulator is the simplest
//! of the included types - it takes a closure/`Fn` to update its value, and it has
//! amplitude and frequency values. Since it uses a closure it can actually make
//! any signal: a waveform, a constant, a random number, etc. For example:
//!
//! ```ignore
//! // Create a sine wave modulator, initial amplitude of 1 and frequency of 0.5Hz
//! let wave = Wave::new(1.0, 0.5).wave(Box::new(|w, t| {
//!         (t * w.frequency * f32::consts::PI * 2.0).sin() * w.amplitude
//!     }));
//!
//! // Give the modulator to the environment
//! st.m1.take("wave_sin", Box::new(wave));
//! ```
//!
//! This creates a wave modulator that produces a sine with amplitude 1 and frequency
//! of 0.5Hz. The closure receives the modulator `w` and elapsed time (t: `f32`) in
//! seconds.
//!
//! Once created, `wave` is _given_ to host `m1` which takes ownership of it and tags
//! it with key `"wave_sin"`.
//!
//! Another example:
//!
//! ```ignore
//! // Create a wave modulator, amplitude (2.0) here is used to define walk bounds,
//! // while frequency (0.1) is the random range the value moves each time it advances
//! let wave = Wave::new(2.0, 0.1).wave(Box::new(|w, _| {
//!     let n = w.value + thread_rng().gen_range(-w.frequency, w.frequency);
//!     f32::min(f32::max(n, -w.amplitude), w.amplitude)
//! }));
//!
//! // Now give the modulator to the environment
//! st.m1.take("wave_rnd", Box::new(wave));
//! ```
//!
//! This closure offsets the modulator's current value each `advance(dt)` by a random
//! offset (set by frequency) and caps it between -/+ amplitude. This creates a
//! simple random walk.
//!
//! Once the modulators above have been created and given to the host, their value can be
//! read anytime as follows:
//!
//! ```ignore
//! let v0 = st.m1.value("wave_sin"); // current value of sine modulator
//! let v1 = st.m1.value("wave_rnd"); // current value of random walk modulator
//! ```
//!
//! **Modulator details**
//! -----
//!
//! Notice that modulators should cache their `value` when they are advanced, which
//! means that, even if advancing could be expensive, reading their value must
//! always be fast. Furthermore, modulators are advanced by the environment all
//! at once to ensure that reading of interdependent values is always consistent.
//!
//! It is **important** to notice that modulators are __not__ guaranteed to be
//! reversible. Most will not be, in fact. They can only evolve _forward_ in time.
//!
//! The reason for this restriction is that, while modulators are generally expected
//! to be _frame rate independent_ (they should express their evolution as a function of
//! time), they are also frequently going to have __discrete state changes__.
//!
//! For example, the included modulator `ScalarGoalFollower` picks a random value, sets
//! it as the _goal_ for a contained sub-modulator, then observes it until it determines
//! that the sub-modulator has arrived to its goal. Once it does, the follower makes
//! a new goal and repeats the process.
//!
//! This kind of discrete-state, randomized behavior would be costly to make reversible
//! and would require caching of the randomly generated values, amongst other problems.
//! Since being reversible is not critical in the vast majority of applications, the
//! `Modulator` trait does not make it a part of its contract - versatility is preferred.
//!
//! Modulators are generally __expected__ to be frame-rate independent, but not required.
//! All of the ones provided with the crate are, even those that include discrete events
//! such as the `ScalarGoalFollower` described above, and they evolve consistently even
//! with varying frame lengths.
//!
//! The `Wave` modulator is a special case, since it uses a closure to compute its value
//! it might or might not be time-based depending on the given function.
//!
//! Recall the "sine wave" closure we gave to `Wave` earlier, its implementation is
//! obviously a function of time (and, in this simple case, it would be reversible too).
//! The "random walk" closure, on the other hand, is neither, as the rate at
//! which the value is updated is a function of the number of times `advance(dt)` is
//! called, rather than elapsed time. A random walk that changes in frequency depending
//! on frame rate would be of limited use, and in production code we would implement
//! a more sophisticated random walk with update rate expressed in changes per second.
//!
//! **Modulator lifetime and interaction**
//! -----
//!
//! A `ModulatorEnv` host only knows two things about the modulators it owns:
//!
//!   1. They implement `Modulator<T>`
//!   2. They have the same `T` (value type)
//!
//! This means that the only operations the environment can perform on its modulators
//! are the ones defined by the `Modulator` trait.
//!
//! While the modulator types provided in `sources.rs` are all designed specifically
//! for their role as modulators, other types can implement the modulator trait and
//! acquire modulation capabilities (although in such cases they probably won't be
//! stored in an _owning_ environment).
//!
//! It is clear that `ModulatorEnv` contents are heterogeneous - the only thing they
//! are known to have in common is that they `impl Modulator<T>` for the same `T` as
//! the environment. This is a proper use case for Rust's **trait objects**, and
//! in fact that's how `ModulatorEnv` stores the modulators it owns.
//!
//! Often modulators are created, added to an environment and then factored into
//! calculations at destination points, addressed by the symbolic name that was
//! given to the host when added. For example:
//!
//! ```ignore
//! // Here we are updating some value by scaling it with a modulator, source
//! // is the name of the modulator in environment m1
//! self.height = self.base + self.range * st.m1.value(source);
//! ```
//!
//! Still, at times you will want to access a modulator out of an environment and
//! modify something about it, perhaps to modulate one of its settings
//! by another modulator.
//!
//! Since `ModulatorEnv` stores its contents as trait objects, borrowing a modulator
//! back requires knowing its type and downcasting it. Suppose we want to modulate the
//! amplitude of our previous `"wave_sin"` modulator by another modulator, in the
//! same environment, called `"amp_mod"`:
//!
//! ```ignore
//! let ampmod = st.m1.value("amp_mod"); // amplitude modulation value
//! if let Some(sw) = st.m1.get_mut("wave_sin") { // borrow trait object
//!     if let Some(ss) = sw.as_any().downcast_mut::<Wave>() { // safely cast it
//!         ss.amplitude = 1.0 + ampmod; // modify its amplitude attribute
//!     }
//! }
//! ```
//!
//! Here, we read the current value of `"amp_mod"` then we mutably borrow a reference
//! to the `"wave_sin"` trait object. The `as_any()` method is part of the `Modulator`
//! trait, so all modulators implement this conversion through a default trait method:
//!
//! ```ignore
//! fn as_any(&mut self) -> &mut Any where Self: 'static + Sized {
//!     self
//! }
//! ```
//!
//! Once the trait object has been converted into an `Any` we use the `downcast_mut`
//! method to safely convert it to its original type, which of course must be known.
//! In the case above, we downcast to `Wave` and then modulate the amplitude of
//! `"wave_sin"` by the current value of `"amp_mod"`.
//!
//! Notice that, while the `ModulatorEnv` type is convenient and useful in a large
//! number of cases, it is not required. Countless alternative approaches to hosting
//! modulators are possible, including not having a dedicated host at all. Modulators
//! only need to be accessible and be advanced appropriately, and `ModulatorEnv` is
//! just one approach to doing so.
//!
//! **Other methods of the `Modulator` trait**
//! -----
//!
//! Besides `value()`, `advance()` and `as_any()` the `Modulator` crate defines several
//! other methods. Mostly these are optional and modulators are not required to
//! implement them in a meaningful manner.  See the trait methods for details, and then
//! the implementation for each of the included modulators.
//!
//! Finally, notice the modulator enabled status methods:
//!
//! ```ignore
//! /// Check if the modulator is disabled
//! fn enabled(&self) -> bool;
//!
//! /// Toggle enabling/disabling the modulator
//! fn set_enabled(&mut self, enabled: bool);
//! ```
//!
//! Notice that `ModulatorEnv` checks the enabled status of its modulators and will
//! __not__ advance them if they are disabled. This allows the pausing/unpausing of
//! modulators.
//!
//! **The included modulators**
//! -----
//!
//! Several modulators are provided in `sources.rs`. Each is documented locally,
//! but we will provide a summary here.
//!
//! 1. **`Wave`**
//!
//! Simple modulator using a value closure/`Fn`, with frequency and amplitude. The
//! closure receives self, elapsed time (in seconds) and returns a new value.
//!
//! 2. **`ScalarSpring`**
//!
//! Critically damped spring modulator. Moves towards its set `goal` with `smooth` seconds
//! of delay, critically damping its arrival so it slows down and stops at the goal without
//! overshooting or oscillation.
//!
//! If overshooting is desired, positive values of `undamp` can be set to add artificial
//! overshoot/oscillations around the goal.
//!
//! 3. **`Newtonian`**
//!
//! A modulator that uses classical mechanics to move to its `goal` - it guarantees smooth
//! acceleration, deceleration and speed limiting regardless of settings.
//!
//! The goal calculation computes an analytical solution to the motion equation. When
//! a new goal is set, `speed_limit`, `acceleration` and `deceleration` values are
//! picked from their respective ranges, then movement begins with the value starting
//! from current value with 0 velocity, accelerating at the selected rate up to the speed
//! limit, then decelerating at the selected rate of deceleration so that it is _guaranteed_
//! to come to a stop at the goal.
//!
//! The analytical solution to the motion equation ensures that, regardless of input, the
//! value always accelerates and decelerates at the picked rates, and never exceeds the
//! speed max. If there is not enough time to reach peak speed, the value accelerates as
//! much as it it can while ensuring that it will decelerate and come to a stop (0 speed)
//! exactly at `goal`.
//!
//! 4. **`ScalarGoalFollower`**
//!
//! A programmable goal follower. Picks a `goal` within one of its `regions` for its owned
//! `follower` modulator, then  monitor its progress until the follower gets to `threshold`
//! distance to the goal and has velocity of `vel_threshold` or less, at which point it
//! considers it arrived.
//!
//! Once a goal has been reached, it picks a pause duration microseconds from `pause_range`,
//! waits for the pause to elapse, then picks a new goal and repeats the process.
//!
//! This modulator can be given any other modulator type as its owned `follower`, but a
//! type that is unable to pursue and arrive to its given `goal` is, of course, never going
//! to satisfy the conditions for arrival.
//!
//! 5. **`ShiftRegister`**
//!
//! Inspired by classic analog shift registers like those used in Buchla synthesizers, this
//! modulator has a vector of values `buckets` containing values selected from `value_range`.
//!
//! A `period` of the register is the length of time, in seconds, that the value takes to
//! visit all the buckets in the register. Once a period is over, the value moves back to
//! the first bucket and continues to move.
//!
//! If `interp` is `ShiftRegisterInterp::None` then the value returned corresponds to the
//! current bucket being visited. If it is `ShiftRegisterInterp::Linear` then it is the
//! linear interpolation of the current bucket and the next. If it is
//! `ShiftRegisterInterp::Quadratic` then the value is the result of polynomial interpolation
//! of the values of the previous, current and next bucket.
//!
//! Every time the value leaves a bucket (it is done visiting it for the period) it has
//! `odds` chances of _replacing_ the value in the bucket it just left, where `odds` ranges
//! from 0.0 (value never changes) to 1.0 (value always changes).
//!
//! Parameter `age_range` can be used to specify an age (in periods) over which the odds
//! of a value changing increase linearly. For example: if `odds` is set to 0.1 (10%) and
//! `age_range` is set to [200, 1000) then for the first 200 periods a value's odds of
//! changing are 10%, and between 200 and 1000 periods they increase from 10% to 100%. By
//! default `age_range` is set to [u32::MAX, u32::MAX] so the odds never change.
//!
//! The result is that the shift register is periodic and exhibits a pattern (given low
//! enough odds), but still evolves over time in an organic way.
//!
//! Copyright© 2018 Ready At Dawn Studios

use std::any::Any;
use std::collections::HashMap;
use std::time::Duration;

pub mod sources;

/// Modulators provide animated modulation. T is the generic type of the modulator value.
pub trait Modulator<T> {
    /// Value of the modulator at the current time.
    fn value(&self) -> T;

    /// Domain of the modulator as min..=max.
    fn range(&self) -> [T; 2];
    /// Total accumulated microseconds for the modulator.
    fn elapsed_us(&self) -> u64;

    /// Allow donwcasting.
    fn as_any(&mut self) -> &mut Any where Self: 'static + Sized {
        self
    }

    /// Check if the modulator is disabled
    fn enabled(&self) -> bool;
    /// Toggle enabling/disabling the modulator
    fn set_enabled(&mut self, enabled: bool);

    /// Current goal of the modulator, if/when meaningful.
    fn goal(&self) -> T;
    /// Set a goal for the modulator to move towards, if possible.
    fn set_goal(&mut self, goal: T);

    /// Move the modulator ahead by dt microseconds.
    fn advance(&mut self, dt: u64);
}

/// A host for modulators, homogeneous in type T for the value of its modulators
#[derive(Default)]
pub struct ModulatorEnv<T> {
    mods: HashMap<String, Box<dyn Modulator<T>>>, // live modulators
}

impl<T: Default> ModulatorEnv<T> {
    /// Create an empty ModulatorEnv
    pub fn new() -> Self {
        ModulatorEnv {
            mods: HashMap::new(),
        }
    }

    /// Given a unique key for the modulator, take ownership and hash it into mods table
    pub fn take(&mut self, key: &str, modulator: Box<dyn Modulator<T>>) {
        self.mods.insert(key.to_string(), modulator);
    }
    /// Remove the modulator with given key, let it die
    pub fn kill(&mut self, key: &str) {
        self.mods.remove(key); // ignore the return, let the value die
    }

    /// Take an immutable reference to the mods table
    pub fn get_mods(&self) -> &HashMap<String, Box<dyn Modulator<T>>> {
        &self.mods
    }

    /// Try to fetch an immutable reference to the modulator with the given  key
    pub fn get(&self, key: &str) -> Option<&Box<dyn Modulator<T>>> {
        self.mods.get(key)
    }
    /// Try to fetch an mutable reference to the modulator with the given  key
    pub fn get_mut(&mut self, key: &str) -> Option<&mut Box<dyn Modulator<T>>> {
        self.mods.get_mut(key)
    }

    /// Return the current value of the given modulator
    pub fn value(&self, key: &str) -> T {
        match self.get(key) {
            Some(modulator) if modulator.enabled() => modulator.value(),
            Some(_) => T::default(),
            None => T::default(),
        }
    }
    /// Return the range of the given modulator
    pub fn range(&self, key: &str) -> [T; 2] {
        match self.get(key) {
            Some(modulator) if modulator.enabled() => modulator.range(),
            Some(_) => [T::default(), T::default()],
            None => [T::default(), T::default()],
        }
    }
    /// Return the current goal of the given modulator
    pub fn goal(&self, key: &str) -> T {
        match self.get(key) {
            Some(modulator) if modulator.enabled() => modulator.goal(),
            Some(_) => T::default(),
            None => T::default(),
        }
    }

    /// Return the current value of the given modulator
    pub fn elapsed_us(&self, key: &str) -> u64 {
        match self.get(key) {
            Some(modulator) => modulator.elapsed_us(),
            None => 0,
        }
    }

    /// Advance all owned modulators by dt microseconds
    pub fn advance(&mut self, dt: u64) {
        for v in self.mods.values_mut() {
            if v.enabled() {
                v.advance(dt);
            }
        }
    }

    /// Convert a duration (secs+nanosecs) into total microseconds
    pub fn duration_to_micros(time: Duration) -> u64 {
        time.as_secs() * 1_000_000_u64 + u64::from(time.subsec_nanos()) / 1000_u64
    }
    /// Convert microseconds into floating point seconds
    pub fn micros_to_secs(us: u64) -> f32 {
        us as f32 / 1.0e6_f32
    }
    /// Convert a duration (secs+nanosecs) into floating point seconds
    pub fn duration_to_secs(time: Duration) -> f32 {
        Self::micros_to_secs(Self::duration_to_micros(time))
    }
}
