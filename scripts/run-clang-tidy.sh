#!/usr/bin/env bash

LAST_ARG=${@:$#}
EXEC_ROOT=`bazel info execution_root`
BAZEL_EXTERNAL=${EXEC_ROOT}/external
BAZEL_THIRD_PARTY=${EXEC_ROOT}/third_party

clang-tidy $@ -header-filter=$(realpath .) -system-headers=0 -p ./ \
           -- \
           -std=c++14 \
           -I./ \
           -x c++ \
           -I bazel-genfiles \
	   -I/opt/libibex/2.7.4/include \
	   -I/opt/libibex/2.7.4/include/ibex \
	   -I/opt/libibex/2.7.4/include/ibex/3rd \
	   -I/usr/local/opt/ibex@2.7.4/include \
	   -I/usr/local/opt/ibex@2.7.4/include/ibex \
	   -I/usr/local/opt/ibex@2.7.4/include/ibex/3rd \
	   -I/usr/local/opt/clp/include/clp/coin \
	   -I/usr/local/opt/eigen/include/eigen3 \
	   -I/usr/local/opt/coinutils/include/coinutils/coin \
	   -I/usr/local/opt/python@2/Frameworks/Python.framework/Versions/2.7/include/python2.7 \
           -isystem ${BAZEL_EXTERNAL}/spdlog/include \
           -isystem ${BAZEL_EXTERNAL}/fmt/include \
           -isystem ${BAZEL_EXTERNAL}/drake_symbolic \
           -isystem ${BAZEL_EXTERNAL}/ezoptionparser \
           -isystem ${BAZEL_EXTERNAL}/com_google_googletest/googletest/include \
           -isystem ${BAZEL_EXTERNAL}/picosat \
           -isystem ${BAZEL_EXTERNAL}/pybind11/include \
           -isystem ${BAZEL_THIRD_PARTY}/com_github_robotlocomotion_drake \
           -isystem /usr/local/opt/llvm/include/c++/v1 \
           -isystem /usr/local/include \
           -isystem /usr/local/opt/flex/include \
           -isystem /usr/local/opt/flex/include \
	   -I/usr/include/eigen3 \

if [ ${LAST_ARG} == "--fix" ];
then
    git-clang-format -f
fi
