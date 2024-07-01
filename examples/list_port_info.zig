const std = @import("std");
const zig_serial = @import("serial");

pub fn main() !u8 {
    var iterator = try zig_serial.list_info();
    defer iterator.deinit();

    while (try iterator.next()) |info| {
        std.debug.print("\nPort name: {s}\n", .{info.port_name});
        std.debug.print(" - System location: {s}\n", .{info.system_location});
        std.debug.print(" - Friendly name: {s}\n", .{info.friendly_name});
        std.debug.print(" - Description: {s}\n", .{info.description});
        std.debug.print(" - Manufacturer: {s}\n", .{info.manufacturer});
        std.debug.print(" - Serial #: {s}\n", .{info.serial_number});
        std.debug.print(" - HW ID: {s}\n", .{info.hw_id});
        std.debug.print(" - VID: 0x{X:0>4} PID: 0x{X:0>4}\n", .{ info.vid, info.pid });
    }

    return 0;
}
