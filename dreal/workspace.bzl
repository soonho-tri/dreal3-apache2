load("//third_party/com_github_robotlocomotion_drake:tools/workspace/github.bzl", "github_archive")
load("//third_party/com_github_robotlocomotion_drake:tools/workspace/pkg_config.bzl", "pkg_config_repository")
load("//third_party/com_github_tensorflow_tensorflow/py:python_configure.bzl", "python_configure")

def dreal_workspace():
    pkg_config_repository(
        name = "ibex",  # LGPL3
        modname = "ibex",
        pkg_config_paths = [
            # macOS
            "/usr/local/opt/ibex@2.7.4/share/pkgconfig",
            "/usr/local/opt/clp/lib/pkgconfig",  # dep of ibex, EPL
            "/usr/local/opt/coinutils/lib/pkgconfig",  # dep of clp, EPL
            # Linux
            "/opt/libibex/2.7.4/share/pkgconfig",
        ],
    )
    pkg_config_repository(
        name = "nlopt",  # LGPL2 + MIT
        modname = "nlopt",
        pkg_config_paths = [
            "/usr/local/opt/nlopt/lib/pkgconfig",
        ],
    )
    python_configure(name = "local_config_python")

    github_archive(
        name = "spdlog",  # MIT
        repository = "gabime/spdlog",
        commit = "v1.3.1",
        sha256 = "160845266e94db1d4922ef755637f6901266731c4cb3b30b45bf41efa0e6ab70",
        build_file = str(Label("//tools:spdlog.BUILD.bazel")),
    )

    github_archive(
        name = "fmt",  # BSD2
        repository = "fmtlib/fmt",
        commit = "5.3.0",
        sha256 = "defa24a9af4c622a7134076602070b45721a43c51598c8456ec6f2c4dbb51c89",
        build_file = str(Label("//tools:fmt.BUILD.bazel")),
    )

    github_archive(
        name = "picosat",  # MIT
        repository = "dreal-deps/picosat",
        commit = "4ee7aa1d1c645df8fa9daa07f2be17c6d03b35fc",  # v965
        sha256 = "1be461d3659d4e3dc957a718ed295941c38dc822fd22a67f4cb5d180f0b6a7a3",
        build_file = str(Label("//tools:picosat.BUILD.bazel")),
    )

    github_archive(
        name = "pybind11",  # BSD
        repository = "pybind/pybind11",
        commit = "v2.2.4",
        sha256 = "b69e83658513215b8d1443544d0549b7d231b9f201f6fc787a2b2218b408181e",
        build_file = str(Label("//tools:pybind11.BUILD.bazel")),
    )

    github_archive(
        name = "com_google_absl",  # BSD
        repository = "abseil/abseil-cpp",
        commit = "d78310fe5a82f2e0e6e16509ef8079c8d7e4674e",  # 20190131
        sha256 = "4c2e4194bbddcb5162933e45fe574d2c4e77a2ef00818b8dac0392459707bfff",
    )

    github_archive(
        name = "cds",  # BSL 1.0
        repository = "khizmax/libcds",
        commit = "v2.3.3",
        sha256 = "f090380ecd6b63a3c2b2f0bdb27260de2ccb22486ef7f47cc1175b70c6e4e388",
        build_file = str(Label("//tools:cds.BUILD.bazel")),
    )

    github_archive(
        name = "cpp_taskflow",  # MIT
        repository = "cpp-taskflow/cpp-taskflow",
        commit = "b52eaf076271e125a89e3fdb574c14d485f8c89a",  # 20190423
        sha256 = "c02dd893e836bb56683c7c0f22bcd2c0007dce42bb42e8b9083cf242a35acb39",
        build_file = str(Label("//tools:cpp_taskflow.BUILD.bazel")),
    )
