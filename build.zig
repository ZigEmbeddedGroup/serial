const std = @import("std");

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    const serial_mod = b.addModule("serial", .{
        .root_source_file = .{ .path = "src/serial.zig" },
    });

    const echo_exe = b.addExecutable(.{
        .name = "serial-echo",
        .root_source_file = .{ .path = "examples/echo.zig" },
        .target = target,
        .optimize = optimize,
    });
    echo_exe.root_module.addImport("serial", serial_mod);
    b.installArtifact(echo_exe);

    const list_exe = b.addExecutable(.{
        .name = "serial-list",
        .root_source_file = .{ .path = "examples/list.zig" },
        .target = target,
        .optimize = optimize,
    });
    list_exe.root_module.addImport("serial", serial_mod);
    b.installArtifact(list_exe);

    // TODO: Linux and MacOS port info support
    const os_tag = list_exe.rootModuleTarget().os.tag;
    if (os_tag == .windows) {
        const port_info_exe = b.addExecutable(.{
            .name = "serial-list-info",
            .root_source_file = .{ .path = "examples/list_port_info.zig" },
            .target = target,
            .optimize = optimize,
        });
        port_info_exe.root_module.addImport("serial", serial_mod);
        b.installArtifact(port_info_exe);
    }
}
