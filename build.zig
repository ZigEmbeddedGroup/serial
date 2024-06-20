const std = @import("std");

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    const serial_mod = b.addModule("serial", .{
        .root_source_file = b.path("src/serial.zig"),
    });

    const unit_tests = b.addTest(.{
        .root_source_file = b.path("src/serial.zig"),
        .target = target,
        .optimize = optimize,
    });
    const run_unit_tests = b.addRunArtifact(unit_tests);
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_unit_tests.step);

    const echo_exe = b.addExecutable(.{
        .name = "serial-echo",
        .root_source_file = b.path("examples/echo.zig"),
        .target = target,
        .optimize = optimize,
    });
    echo_exe.root_module.addImport("serial", serial_mod);
    b.installArtifact(echo_exe);

    const list_exe = b.addExecutable(.{
        .name = "serial-list",
        .root_source_file = b.path("examples/list.zig"),
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
            .root_source_file = b.path("examples/list_port_info.zig"),
            .target = target,
            .optimize = optimize,
        });
        port_info_exe.root_module.addImport("serial", serial_mod);
        b.installArtifact(port_info_exe);
    }
}
