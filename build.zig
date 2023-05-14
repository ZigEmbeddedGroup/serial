const std = @import("std");

pub fn build(b: *std.build.Builder) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    const serial_mod = b.addModule("serial", .{
        .source_file = .{ .path = "src/serial.zig" },
    });

    const echo_exe = b.addExecutable(.{
        .name = "serial-echo",
        .root_source_file = .{ .path = "examples/echo.zig" },
        .target = target,
        .optimize = optimize,
    });
    echo_exe.addModule("serial", serial_mod);
    b.installArtifact(echo_exe);

    const list_exe = b.addExecutable(.{
        .name = "serial-list",
        .root_source_file = .{ .path = "examples/list.zig" },
        .target = target,
        .optimize = optimize,
    });
    list_exe.addModule("serial", serial_mod);
    b.installArtifact(list_exe);
}
