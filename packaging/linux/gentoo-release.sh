#!/bin/sh

# exit when any command fails
set -e

PKG_NAME=
PKG_VERSION=0
PKG_CATEGORY=

while test $# -gt 0
do
    case "$1" in
        --pkg-name) PKG_NAME="$2"
            ;;
        --pkg-version) PKG_VERSION="$2"
            ;;
        --pkg-category) PKG_CATEGORY="$2"
            ;;
    esac
    shift
done

emerge --sync

git clone "git@gitlab.com:inivation/gentoo-inivation.git"

cp ${PKG_NAME}-gentoo.ebuild gentoo-inivation/${PKG_CATEGORY}/${PKG_NAME}/${PKG_NAME}-${PKG_VERSION}.ebuild
cd gentoo-inivation/${PKG_CATEGORY}/${PKG_NAME}/
ebuild ${PKG_NAME}-${PKG_VERSION}.ebuild manifest

git add ${PKG_NAME}-${PKG_VERSION}.ebuild Manifest
git commit -m "${PKG_CATEGORY}/${PKG_NAME}: new release ${PKG_VERSION}."
git push
