const std = @import("std");
const math = std.math;
const vectors = @import("vectors.zig");

const Vector3 = vectors.Vector(3, f64);

pub const BodyDescription = struct {
    name: []const u8,
    mu: f64,
    initial_position: Vector3,
    initial_velocity: Vector3,
};

const BodyInfo = struct {
    // `id` must always be the index into the containing array and match id in
    // corresponding `BodyState`.
    id: usize,
    name: []const u8,
    /// Standard gravitational parameter
    mu: f64,

    pub fn from(description: BodyDescription, id: usize) BodyInfo {
        return @This(){
            .id = id,
            .name = description.name,
            .mu = description.mu,
        };
    }
};

const BodyState = struct {
    // `id` must always be the index into the containing array and match id in
    // corresponding `BodyInfo`.
    id: usize,
    position: Vector3,
    velocity: Vector3,

    pub fn from(description: BodyDescription, id: usize) BodyState {
        return @This(){
            .id = id,
            .position = description.initial_position,
            .velocity = description.initial_velocity,
        };
    }

    pub fn get_derivative(self: @This(), acceleration: Vector3) BodyDelta {
        return BodyDelta{
            .of_position = self.velocity,
            .of_velocity = acceleration,
        };
    }

    pub fn apply_derivative_in_place(self: *@This(), derivative: BodyDelta, dt: f64) void {
        const delta = derivative.scale_by(dt);
        self.position = self.position.plus(delta.of_position);
        self.velocity = self.velocity.plus(delta.of_velocity);
    }
};

const BodyDelta = struct {
    of_position: Vector3,
    of_velocity: Vector3,

    pub fn add(self: @This(), rhs: @This()) @This() {
        return @This(){
            .of_position = self.of_position.plus(rhs.of_position),
            .of_velocity = self.of_velocity.plus(rhs.of_velocity),
        };
    }

    pub fn scale_by(self: @This(), factor: f64) @This() {
        return @This(){
            .of_position = self.of_position.scale_by(factor),
            .of_velocity = self.of_velocity.scale_by(factor),
        };
    }
};

pub const IntegrationStrategy = enum { Euler, IsolatedRK4, GlobalRK4 };

fn Epoch(comptime n: usize) type {
    return struct {
        index: usize,
        body_states: [n]BodyState,
    };
}

pub fn System(comptime n: usize) type {
    return struct {
        _body_info: [n]BodyInfo,
        _epoch: Epoch(n),

        fn compute_gravitational_acceleration(self: @This(), body_id: usize) Vector3 {
            const epoch = self._epoch;

            const body_state = epoch.body_states[body_id];
            var acceleration = std.mem.zeroes(Vector3);
            for (epoch.body_states) |other_body_state, other_idx| {
                // Skip the selected body itself
                if (other_body_state.id == body_state.id) {
                    continue;
                }
                const from_body_to_other_body: Vector3 = other_body_state.position.minus(body_state.position);
                acceleration = acceleration.plus(from_body_to_other_body.unit().scale_by(self._body_info[other_idx].mu / from_body_to_other_body.squared_magnitude()));
            }
            return acceleration;
        }

        fn compute_all_gravitational_accelerations(self: @This()) [n]Vector3 {
            const epoch = self._epoch;
            var accelerations = std.mem.zeroes([n]Vector3);
            for (epoch.body_states) |body_state, idx| {
                for (epoch.body_states[(idx + 1)..]) |other_body_state, shifted_idx| {
                    const other_idx = shifted_idx + 1;
                    const from_body_to_other_body: Vector3 = other_body_state.position.minus(body_state.position);
                    const normalized_field = from_body_to_other_body.unit().scale_by(1.0 / from_body_to_other_body.squared_magnitude());
                    accelerations[idx] = accelerations[idx].plus(normalized_field.scale_by(self._body_info[other_idx].mu));
                    accelerations[other_idx] = accelerations[other_idx].plus(normalized_field.scale_by(-self._body_info[idx].mu));
                }
            }
            return accelerations;
        }

        fn compute_derivatives(self: @This(), accelerations: [n]Vector3) [n]BodyDelta {
            const epoch = self._epoch;
            var derivatives: [n]BodyDelta = undefined;
            for (epoch.body_states) |body_state, idx| {
                derivatives[idx] = body_state.get_derivative(accelerations[idx]);
            }
            return derivatives;
        }

        fn apply_derivatives_in_place(self: *@This(), derivatives: [n]BodyDelta, dt: f64) void {
            for (derivatives) |delta, idx| {
                self._epoch.body_states[idx].apply_derivative_in_place(delta, dt);
            }
        }

        pub fn from(descriptions: [n]BodyDescription) @This() {
            var self = @This(){ ._body_info = undefined, ._epoch = Epoch(n){
                .index = 0,
                .body_states = undefined,
            } };

            for (descriptions) |description, idx| {
                self._body_info[idx] = BodyInfo.from(description, idx);
                self._epoch.body_states[idx] = BodyState.from(description, idx);
            }
            return self;
        }

        pub fn step(self: *@This(), strategy: IntegrationStrategy, dt: f64) void {
            // Stack allocate scratch space for computing Runge-Kutta
            // system states and final epoch state.
            const last_epoch = self._epoch;

            switch (strategy) {
                .Euler => {
                    for (last_epoch.body_states) |_, idx| {
                        const accel = self.compute_gravitational_acceleration(idx);
                        const derivative = self._epoch.body_states[idx].get_derivative(accel);
                        self._epoch.body_states[idx].apply_derivative_in_place(derivative, dt);
                    }
                },
                .IsolatedRK4 => {
                    // Perform RK4 integration for each body in isolation.
                    // This means each bodies movement during t=t0 to
                    // t=t0+dt will not impact any other's movement.
                    // This could be optimized slightly more by calculating all
                    // the forces for the first iterations at once (and only once
                    // per pair).
                    for (last_epoch.body_states) |_, idx| {
                        // 0th iteration, f(0) computed and applied from 0 to dt/2
                        const k0_accel = self.compute_gravitational_acceleration(idx);
                        const k0_derivative = self._epoch.body_states[idx].get_derivative(k0_accel);
                        self._epoch.body_states[idx].apply_derivative_in_place(k0_derivative, dt / 2.0);

                        // 1st iteration, f(dt/2)_0 computed and applied from 0 to dt/2
                        const k1_accel = self.compute_gravitational_acceleration(idx);
                        const k1_derivative = self._epoch.body_states[idx].get_derivative(k1_accel);
                        self._epoch.body_states[idx] = last_epoch.body_states[idx]; // Reset the 0th iteration mutations
                        self._epoch.body_states[idx].apply_derivative_in_place(k1_derivative, dt / 2.0);

                        // 2nd iteration, f(dt/2)_1 computed and applied from 0 to dt
                        const k2_accel = self.compute_gravitational_acceleration(idx);
                        const k2_derivative = self._epoch.body_states[idx].get_derivative(k2_accel);
                        self._epoch.body_states[idx] = last_epoch.body_states[idx]; // Reset the 1th iteration mutations
                        self._epoch.body_states[idx].apply_derivative_in_place(k2_derivative, dt);

                        // 3rd iteration, f(dt)_0 computed
                        const k3_accel = self.compute_gravitational_acceleration(idx);
                        const k3_derivative = self._epoch.body_states[idx].get_derivative(k3_accel);

                        // f(0..dt)_rk computed and applied from 0 to dt
                        var rk_derivative: BodyDelta = k0_derivative.scale_by(1.0 / 6.0)
                            .add(k1_derivative.scale_by(2.0 / 6.0))
                            .add(k2_derivative.scale_by(2.0 / 6.0))
                            .add(k3_derivative.scale_by(1.0 / 6.0));

                        self._epoch.body_states[idx] = last_epoch.body_states[idx]; // Reset the 2nd iteration mutations
                        self._epoch.body_states[idx].apply_derivative_in_place(rk_derivative, dt);
                    }

                    self._epoch.index += 1;
                },
                .GlobalRK4 => {
                    // Perform global RK4 integration.
                    // This properly handles the mutual impacts of intermediate
                    // states, but requires significantly more copy operations
                    // to preserve the proper initial epoch state for
                    // intermediate calculations.
                    // Over all this is actually still faster than IsolatedRK4
                    // because the fact that all planets update together means
                    // we only have to calculate the acceleration due to gravity
                    // once per pair, as opposed to once per body.

                    // 0th iteration, f(0) computed and applied from 0 to dt/2
                    const k0_accel = self.compute_all_gravitational_accelerations();
                    const k0_derivatives = self.compute_derivatives(k0_accel);
                    self.apply_derivatives_in_place(k0_derivatives, dt / 2.0);

                    // 1st iteration, f(dt/2)_0 computed and applied from 0 to dt/2
                    const k1_accel = self.compute_all_gravitational_accelerations();
                    const k1_derivatives = self.compute_derivatives(k1_accel);
                    self._epoch = last_epoch; // Reset the 0th iteration mutations
                    self.apply_derivatives_in_place(k1_derivatives, dt / 2.0);

                    // 2nd iteration, f(dt/2)_1 computed and applied from 0 to dt
                    const k2_accel = self.compute_all_gravitational_accelerations();
                    const k2_derivatives = self.compute_derivatives(k2_accel);
                    self._epoch = last_epoch; // Reset the 1st iteration mutations
                    self.apply_derivatives_in_place(k2_derivatives, dt);

                    // 3rd iteration, f(dt)_0 computed
                    const k3_accel = self.compute_all_gravitational_accelerations();
                    const k3_derivatives = self.compute_derivatives(k3_accel);

                    // f(0..dt)_rk computed and applied from 0 to dt
                    var rk_derivatives: [n]BodyDelta = undefined;
                    for (rk_derivatives) |*derivative, idx| {
                        derivative.* = k0_derivatives[idx].scale_by(1.0 / 6.0)
                            .add(k1_derivatives[idx].scale_by(2.0 / 6.0))
                            .add(k2_derivatives[idx].scale_by(2.0 / 6.0))
                            .add(k3_derivatives[idx].scale_by(1.0 / 6.0));
                    }
                    self._epoch = last_epoch; // Reset the 2nd iteration mutations
                    self.apply_derivatives_in_place(rk_derivatives, dt);

                    self._epoch.index += 1;
                },
            }
        }
    };
}
