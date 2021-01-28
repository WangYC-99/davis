#!/bin/sh

# exit when any command fails
set -e

PKG_NAME=
PKG_VERSION=0

while test $# -gt 0
do
    case "$1" in
        --pkg-name) PKG_NAME="$2"
            ;;
        --pkg-version) PKG_VERSION="$2"
            ;;
    esac
    shift
done

PKG_FILE="${PKG_NAME}-${PKG_VERSION}.tar.gz"
PKG_FILE_PATH="/var/cache/distfiles/${PKG_FILE}"

echo "Path is: ${PKG_FILE_PATH}"

if ! [ -f "${PKG_FILE_PATH}" ] ; then
	mkdir -p /var/cache/distfiles/
	wget "https://gitlab.com/inivation/dv/${PKG_NAME}/-/archive/${PKG_VERSION}/${PKG_FILE}" -O "${PKG_FILE_PATH}"
fi

SHA256_FILE=$(sha256sum "${PKG_FILE_PATH}" | awk '{ print $1 }')

echo "SHA256 checksum is: ${SHA256_FILE}"

git clone "ssh://aur@aur.archlinux.org/${PKG_NAME}.git"

sed -e "s|VERSION_REPLACE|${PKG_VERSION}|g" -i ${PKG_NAME}-arch.pkgbuild
sed -e "s|SHA256SUM_REPLACE|${SHA256_FILE}|g" -i ${PKG_NAME}-arch.pkgbuild
cp ${PKG_NAME}-arch.pkgbuild ${PKG_NAME}/PKGBUILD

cd ${PKG_NAME}/
chmod 0777 .
sudo -u nobody makepkg --printsrcinfo > .SRCINFO
git add PKGBUILD .SRCINFO
git commit -m "${PKG_NAME}: new release ${PKG_VERSION}."
git push
