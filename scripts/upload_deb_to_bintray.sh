#!/usr/bin/env bash

set -euf -o pipefail

display_usage() {
    echo "usage: $0 <id> <password>"
}

if [ "$(lsb_release -r -s)" != "16.04" ]
then
    echo "Please run this script on Ubuntu 16.04"
    exit 1
fi


if [ $# -ne 2 ]
then
    display_usage
    exit 1
fi

# BUILD
CC=gcc-5 bazel build //:package_debian

ID=$1
PASSWORD=$2
WORKSPACE=$(bazel info workspace)
VERSION=$(grep "DREAL_VERSION_FULL" "${WORKSPACE}"/bazel-bin/dreal/version.h | tr -s " " | cut -d ' ' -f 3-)
DEB_FILENAME=dreal_${VERSION}_amd64.deb
DEB_FILEPATH=${WORKSPACE}/bazel-bin/${DEB_FILENAME}
BINTRAY_URL=https://api.bintray.com/content/dreal/dreal/dreal

if [ -e "${DEB_FILEPATH}" ]
then
  # Upload Files
  echo "Uploading ${DEB_FILEPATH} => ${BINTRAY_URL}/${VERSION}/${DEB_FILENAME} ..."
  curl -T "${DEB_FILEPATH}" -u"${ID}:${PASSWORD}" "${BINTRAY_URL}/${VERSION}/${DEB_FILENAME}"
  # Publish version
  echo "Publishing ${BINTRAY_URL}/${VERSION} ..."
  curl -X POST -u"${ID}:${PASSWORD}" "${BINTRAY_URL}/${VERSION}/publish"
else 
  echo "File not found: ${DEB_FILEPATH}"
fi

