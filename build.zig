const std = @import("std");

pub fn build(b: *std.Build) void {
    const mode = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    b.addModule(.{
        .name = "serial",
        .source_file = .{ .path = "src/serial.zig" },
    });

    const serial_module = b.modules.get("serial") orelse unreachable;

    const echo_exe = b.addExecutable(.{
        .name = "serial-echo",
        .root_source_file = .{ .path = "examples/echo.zig" },
        .target = target,
        .optimize = mode,
    });
    echo_exe.addModule("serial", serial_module);
    echo_exe.install();

    const list_exe = b.addExecutable(.{
        .name = "serial-list",
        .root_source_file = .{ .path = "examples/list.zig" },
        .target = target,
        .optimize = mode,
    });
    list_exe.addModule("serial", serial_module);
    list_exe.install();
}
