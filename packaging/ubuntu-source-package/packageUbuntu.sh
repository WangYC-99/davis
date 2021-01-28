#!/bin/sh

# Requirements: Ubuntu, GPG key (seahorse), devscripts, build-essential

# exit when any command fails
set -e

GPG_KEY_ID=
GPG_KEY_PASS=
PKG_NAME=
PKG_VERSION=0
PKG_RELEASE=1
DISTRO=focal
UPLOAD="false"
DEBUILD_ARGS="-S -sa -d -us -uc"

while test $# -gt 0
do
    case "$1" in
        --pkg-name) PKG_NAME="$2"
            ;;
        --pkg-version) PKG_VERSION="$2"
            ;;
        --distro) DISTRO="$2"
            ;;
        --gpg-key-id) GPG_KEY_ID="$2"
            ;;
        --gpg-key-pass) GPG_KEY_PASS="$2"
            ;;
        --upload) UPLOAD="true"
            ;;
    esac
    shift
done

SRC_URI="https://gitlab.com/inivation/dv/$PKG_NAME/-/archive/$PKG_VERSION/$PKG_NAME-$PKG_VERSION.tar.gz"
PPA_REPO="inivation-ppa/inivation"

if [ "${DISTRO}" = "xenial" ] ; then
	PPA_REPO="inivation-ppa/inivation-xenial"
fi

DATE=$(LC_ALL=C date +'%a, %d %b %Y %T %z')
CUR_DIR=$(pwd)
BASE_DIR="$CUR_DIR/../../"
BUILD_DIR="$CUR_DIR/build/"
PKG_BUILD_DIR="$BUILD_DIR/$PKG_NAME-$PKG_VERSION/"
DEBIAN_DIR="$PKG_BUILD_DIR/debian/"

echo "Started the debian source packaging process for distro $DISTRO"

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

# Get and extract the tar.gz containing the source
cd "$BUILD_DIR"
wget "$SRC_URI" -O "${PKG_NAME}_${PKG_VERSION}.orig.tar.gz"
tar -xvzf "${PKG_NAME}_${PKG_VERSION}.orig.tar.gz"

mkdir -p "$DEBIAN_DIR"

# Copy correct debian build files for distro
cp "$CUR_DIR/$DISTRO/"* "$DEBIAN_DIR/"

# Copy copyright file (use main license)
cp "$BASE_DIR/COPYING" "$DEBIAN_DIR/copyright"

# Formats file
mkdir -p "$DEBIAN_DIR/source/"
echo "3.0 (quilt)" > "$DEBIAN_DIR/source/format"

# Create the changelog file for the distro
CHANGELOG_FILE="$DEBIAN_DIR/changelog"
echo "$PKG_NAME ($PKG_VERSION-$PKG_RELEASE~$DISTRO) $DISTRO; urgency=low" > "$CHANGELOG_FILE"
echo "" >> "$CHANGELOG_FILE"
echo "  * Released $PKG_NAME version $PKG_VERSION for distro $DISTRO." >> "$CHANGELOG_FILE"
echo "" >> "$CHANGELOG_FILE"
echo " -- iniVation AG <support@inivation.com>  $DATE" >> "$CHANGELOG_FILE"

# Launch debuild
cd "$PKG_BUILD_DIR"
debuild $DEBUILD_ARGS

# Sign package
cd "$BUILD_DIR"
debsign -p"gpg --pinentry-mode loopback --passphrase $GPG_KEY_PASS" -S -k"$GPG_KEY_ID" \
  "${PKG_NAME}_${PKG_VERSION}-${PKG_RELEASE}~${DISTRO}_source.changes"

# Send to Launchpad PPA
if [ "$UPLOAD" = "true" ]; then
	dput "ppa:$PPA_REPO" "${PKG_NAME}_${PKG_VERSION}-${PKG_RELEASE}~${DISTRO}_source.changes"
fi;
