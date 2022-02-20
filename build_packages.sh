#!/usr/bin/env bash

# change directory to script root
cd $(dirname $(realpath $0))

if [ -z $1 ]; then
    # all packages
    PKGRANGE=""
    PKGRANGEMSG=""
else
    if [ ! -z $2 ]; then
        # package range from ... to
        PKGRANGE="--packages-skip-up-to $1 --packages-up-to $2"
        PKGRANGEMSG="'$1' ... '$2'"
    else
        # do not skip packages
        PKGRANGE="--packages-up-to $1"
        PKGRANGEMSG="up to '$1'"
    fi
fi

# get number of packages in subset
NPKGS=$(colcon list --names-only $PKGRANGE | wc -l)

echo "building packages $PKGRANGEMSG ($NPKGS)"
colcon build --merge-install $PKGRANGE
