const std = @import("std");
const io = std.io;
const fs = std.fs;
const os = std.os;

const system = @import("system.zig");

pub const Log = struct {
    const BufferedWriter = io.BufferedWriter(4096, fs.File.Writer);
    file: fs.File,
    buffer: BufferedWriter,

    pub fn new(name: []const u8) !@This() {
        const cwd = fs.cwd();
        var file = try cwd.createFile(name, .{ .truncate = true });
        var buffer = BufferedWriter{ .unbuffered_writer = file.writer() };
        _ = try buffer.write("EPOCH,BODY,XPOS,YPOS,ZPOS,XVEL,YVEL,ZVEL\n");

        return @This(){
            .file = file,
            .buffer = buffer,
        };
    }

    pub fn log_body(self: *@This(), epoch_index: usize, body_info: anytype, body_state: anytype) !void {
        try std.fmt.format(
            self.buffer.writer(),
            "{},{s},{},{},{},{},{},{}\n",
            .{
                epoch_index,
                body_info.name,
                body_state.position.data[0],
                body_state.position.data[1],
                body_state.position.data[2],
                body_state.velocity.data[0],
                body_state.velocity.data[1],
                body_state.velocity.data[2],
            },
        );
    }

    pub fn log_system(self: *@This(), simulation: anytype) os.WriteError!void {
        for (simulation._body_info) |body_info, idx| {
            const body_state = simulation._epoch.body_states[idx];

            try self.log_body(simulation._epoch.index, body_info, body_state);
        }
    }

    // Once finalized the output file handle is in an undefined state and the
    // log must not be used.
    pub fn finalize(self: *@This()) !void {
        try self.buffer.flush();
        self.file.close();
    }
};
