#!/bin/sh

# Requirements: Fedora, fedora-packager

# exit when any command fails
set -e

PKG_NAME=
PKG_VERSION=0
FEDORA_RELEASE=f31

while test $# -gt 0
do
    case "$1" in
        --pkg-name) PKG_NAME="$2"
            ;;
        --pkg-version) PKG_VERSION="$2"
            ;;
        --fedora-release) FEDORA_RELEASE="$2"
            ;;
    esac
    shift
done

# Set version number.
sed -e "s|VERSION_REPLACE|${PKG_VERSION}|" -i ${PKG_NAME}.spec

# Download source tarball.
spectool -g ${PKG_NAME}.spec

# Generate source RPM.
fedpkg --release "${FEDORA_RELEASE}" srpm

# Run checks on generated RPM.
fedpkg --release "${FEDORA_RELEASE}" lint

# Now you can upload the source RPM to COPR for building.
