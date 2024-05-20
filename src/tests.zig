const std = @import("std");
const testing = std.testing;
const serial = @import("serial.zig");

// Using com0com to emulate ports
// COM100 is linked to COM200

test "iterator" {
    std.debug.print("\n\tLooking for COM100 and COM200\n",.{});
    var iterator = try serial.iterator();
    var found_100 = false;
    var found_200 = false;
    while (try iterator.next()) |port| {
        std.debug.print("\t{s}\n", .{port.display_name});
        if (std.mem.eql(u8, port.display_name, "COM100")) {
            found_100 = true;
        } else if (std.mem.eql(u8, port.display_name, "COM200")) {
            found_200 = true;
        }
        if (found_100 and found_200) { break; }
    }
    try testing.expect(found_100);
}

test "info iterator" {
    std.debug.print("\n\tPort Friendly Names:\n", .{});
    var iterator = try serial.infoIterator();
    while (try iterator.next()) |port| {
        std.debug.print("\t{s}\n", .{port.friendly_name});
    }
}

test "loopback" {
    const hello = "Hello, World!";
    var port1 = try serial.init("COM100", .{
        .baud_rate = .B115200,
        .handshake = .none,
        .parity = .none,
        .stop_bits = .one,
        .word_size = .eight,
    });
    defer port1.close();

    var port2 = try serial.init("COM200", .{
        .baud_rate = .B115200,
        .handshake = .none,
        .parity = .none,
        .stop_bits = .one,
        .word_size = .eight,
    });
    defer port2.close();

    std.debug.print("\n\tWriting \"{s}\" to port1\n", .{hello});
    try port1.writeAll("Hello, World!");

    var buffer: [13]u8 = undefined;
    _ = try port2.readAll(&buffer);
    std.debug.print("\tRead \"{s}\" from port2\n", .{&buffer});

    try testing.expectEqualStrings("Hello, World!", &buffer);
}

test "flush input" {
    const hello = "Hello, World!";
    const purged = "Buffer was purged!";
    var port1 = try serial.init("COM100", .{
        .baud_rate = .B115200,
        .handshake = .none,
        .parity = .none,
        .stop_bits = .one,
        .word_size = .eight,
    });
    defer port1.close();

    var port2 = try serial.init("COM200", .{
        .baud_rate = .B115200,
        .handshake = .none,
        .parity = .none,
        .stop_bits = .one,
        .word_size = .eight,
    });
    defer port2.close();

    std.debug.print("\n\tWriting \"{s}\" to port1\n", .{hello});
    try port1.writeAll("Hello, World!");

    std.debug.print("\tPurging port2 input buffer\n", .{});
    try serial.flush(port2, .input);

    std.debug.print("\tWriting \"{s}\" to port1\n", .{purged});
    try port1.writeAll(purged);

    var buffer: [purged.len]u8 = undefined;
    _ = try port2.readAll(&buffer);
    std.debug.print("\tRead \"{s}\" from port2\n", .{&buffer});

    try testing.expectEqualStrings(purged, &buffer);
}

// I'm not sure how to go about doing this one.
// test "flush output" {
//     const hello = "Hello, World!";
//     const purged = "Buffer was purged!";
//     var port1 = try serial.init("COM100", .{
//         .baud_rate = .B9600,
//         .handshake = .none,
//         .parity = .none,
//         .stop_bits = .one,
//         .word_size = .eight,
//     });
//     defer port1.close();

//     var port2 = try serial.init("COM200", .{
//         .baud_rate = .B9600,
//         .handshake = .none,
//         .parity = .none,
//         .stop_bits = .one,
//         .word_size = .eight,
//     });
//     defer port2.close();

//     std.debug.print("\n\tWriting \"{s}\" to port1\n", .{hello});
//     try port1.writeAll("Hello, World!");

//     try serial.flush(port1, .output);
//     try port1.writeAll(purged);

//     var buffer: [purged.len]u8 = undefined;
//     _ = try port2.readAll(&buffer);
//     std.debug.print("\tRead \"{s}\" from port2\n", .{&buffer});

//     try testing.expectEqualStrings(purged, &buffer);
// }