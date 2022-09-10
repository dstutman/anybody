const std = @import("std");
const value_of = @import("constants.zig");
const vectors = @import("vectors.zig");
const system = @import("system.zig");
const logger = @import("logger.zig");

const Vector3 = vectors.Vector(3, f64);

pub const log_level: std.log.Level = .info;

pub fn main() anyerror!void {
    var simulation = system.System(8).from(.{
        system.BodyDescription{
            .name = "Sun",
            .mu = value_of.sun_mu,
            .initial_position = Vector3.new(.{ 0.0, 0.0, 0.0 }),
            .initial_velocity = Vector3.new(.{ 0.0, 0.0, 0.0 }),
        },
        system.BodyDescription{
            .name = "Earth",
            .mu = value_of.earth_mu,
            .initial_position = Vector3.new(.{ value_of.au, 0.0, 0.0 }),
            .initial_velocity = Vector3.new(.{ 0.0, 30E3, 0.0 }),
        },
        system.BodyDescription{
            .name = "Moon",
            .mu = value_of.moon_mu,
            .initial_position = Vector3.new(.{ value_of.au + 385000E3, 0.0, 0.0 }),
            .initial_velocity = Vector3.new(.{ 0.0, 30E3 + 1.022E3, 0.0 }),
        },
        system.BodyDescription{
            .name = "Moon_Two",
            .mu = value_of.moon_mu,
            .initial_position = Vector3.new(.{ value_of.au - 385000E3, 0.0, 0.0 }),
            .initial_velocity = Vector3.new(.{ 0.0, 30E3 - 1.022E3, 0.0 }),
        },
        system.BodyDescription{
            .name = "Moon_Three",
            .mu = value_of.moon_mu,
            .initial_position = Vector3.new(.{ value_of.au, 385000E3, 0.0 }),
            .initial_velocity = Vector3.new(.{ -1.022E3, 30E3, 0.0 }),
        },
        system.BodyDescription{
            .name = "Moon_Four",
            .mu = value_of.moon_mu,
            .initial_position = Vector3.new(.{ value_of.au, -385000E3, 0.0 }),
            .initial_velocity = Vector3.new(.{ 1.022E3, 30E3 + 1.022E3, 0.0 }),
        },
        system.BodyDescription{
            .name = "Moon_Five",
            .mu = value_of.moon_mu,
            .initial_position = Vector3.new(.{ value_of.au, 0.0, 385000E3 }),
            .initial_velocity = Vector3.new(.{ -1.022E3, 30E3, 0.0 }),
        },
        system.BodyDescription{
            .name = "Moon_Six",
            .mu = value_of.moon_mu,
            .initial_position = Vector3.new(.{ value_of.au, 0.0, -385000E3 }),
            .initial_velocity = Vector3.new(.{ 1.022E3, 30E3 + 1.022E3, 0.0 }),
        },
    });

    const log_name = "log.csv";
    const log_every = 100;
    const dt = 60.0;
    const final_t = 86400.0 * 365.0 * 50.0;
    const steps = @floatToInt(usize, @ceil(final_t / dt));

    std.log.info("Starting Anybody", .{});

    var log = try logger.Log.new(log_name);
    defer std.log.info("Closing log", .{});
    defer log.finalize() catch unreachable;

    std.log.info("Writing log to: {s}", .{log_name});

    var progress = std.Progress{};
    progress.refresh_rate_ns = 30E6;

    const node = progress.start("Simulation epoch", steps);
    defer node.end();

    var i: usize = 0;
    while (i < steps) {
        if (i % log_every == 0) {
            try log.log_system(simulation);
        }
        simulation.step(.GlobalRK4, dt);
        i += 1;
        node.completeOne();
    }
}
