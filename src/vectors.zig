const std = @import("std");
const math = std.math;

pub fn Vector(comptime len: comptime_int, comptime Element: type) type {
    return struct {
        data: @Vector(len, Element),

        pub fn new(data: [len]Element) @This() {
            return @This(){ .data = data };
        }

        pub fn plus(self: @This(), rhs: @This()) @This() {
            return @This(){ .data = self.data + rhs.data };
        }

        pub fn minus(self: @This(), rhs: @This()) @This() {
            return @This(){ .data = self.data - rhs.data };
        }

        pub fn scale_by(self: @This(), factor: Element) @This() {
            return @This().new(@splat(len, factor) * self.data);
        }

        pub fn dotted_with(self: @This(), rhs: @This()) Element {
            return @reduce(.Add, self.data * rhs.data);
        }

        pub fn squared_magnitude(self: @This()) Element {
            return self.dotted_with(self);
        }

        pub fn magnitude(self: @This()) Element {
            return math.sqrt(self.squared_magnitude());
        }

        pub fn unit(self: @This()) @This() {
            return self.scale_by(1 / self.magnitude());
        }
    };
}

const testing = std.testing;

test "arithmetic" {
    const Vector2 = Vector(2, f32);

    const v1 = Vector2.new(.{ 1.0, 2.0 });
    const v2 = Vector2.new(.{ 2.0 * 1.0, 2.0 * 2.0 });
    const v3 = Vector2.new(.{ 3.0, 6.0 });
    const v4 = Vector2.new(.{ -1.0, -2.0 });

    try testing.expectEqual(v3, v2.plus(v1));
    try testing.expectEqual(v4, v1.minus(v2));
    try testing.expectEqual(v2, v1.scale_by(2.0));
}

test "dot product" {
    const Vector2 = Vector(2, f32);

    const v1 = Vector2.new(.{ 1.0, 0.0 });
    const v2 = Vector2.new(.{ 0.0, 1.0 });
    const v3 = Vector2.new(.{ 1.0, 1.0 });

    try testing.expectEqual(v1.dotted_with(v2), 0.0);
    try testing.expectEqual(v1.dotted_with(v3), 1.0);
    try testing.expectEqual(v2.dotted_with(v3), 1.0);
}

test "magnitude" {
    const Vector2 = Vector(2, f32);

    const v = Vector2.new(.{ 3.0, 4.0 });

    try testing.expectEqual(v.magnitude(), 5.0);
}

test "unit" {
    const Vector2 = Vector(2, f32);

    const v1 = Vector2.new(.{ 1.0, 1.0 }).unit();
    const v2 = Vector2.new(.{ 1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0) });

    try testing.expectEqual(v1, v2);
}
