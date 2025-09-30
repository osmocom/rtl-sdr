#!/usr/bin/env bash
# jenkins build helper script for openbsc.  This is how we build on jenkins.osmocom.org

if ! [ -x "$(command -v osmo-build-dep.sh)" ]; then
	echo "Error: We need to have scripts/osmo-deps.sh from http://git.osmocom.org/osmo-ci/ in PATH !"
	exit 2
fi


set -ex

base="$PWD"
deps="$base/deps"
inst="$deps/install"
export deps inst

osmo-clean-workspace.sh

mkdir "$deps" || true

set +x
echo
echo
echo
echo " =============================== rtl-sdr ==============================="
echo
set -x

cd "$base"
autoreconf --install --force
./configure --enable-sanitize --enable-werror
$MAKE $PARALLEL_MAKE
LD_LIBRARY_PATH="$inst/lib" $MAKE check \
  || cat-testlogs.sh

# Prepare to run "make distcheck" by first running configure without
# --enable-werror. "make distcheck" runs configure again, and if -Werror is set
# the configure script itself will fail build its AC_CHECK_LIB test program,
# resulting in "checking for atan2 in -lm" -> "result: no" and then attempting
# to link rtl_test without -lm, which then fails.
# https://bug-libtool.gnu.narkive.com/YrSM40bs/lt-lib-m-and-gcc
./configure

LD_LIBRARY_PATH="$inst/lib" \
  DISTCHECK_CONFIGURE_FLAGS="--enable-werror" \
  $MAKE distcheck \
  || cat-testlogs.sh

$MAKE maintainer-clean

osmo-clean-workspace.sh
