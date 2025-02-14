#!/bin/bash

INSTALL_USER=nvidia
PATH_RESOURCES_DIR=.
PATH_APPS=/home/${INSTALL_USER}/IB2/others

cd $(dirname ${0})

# Check the execution user
if [ "${USER}" != "root" ]; then
    echo "[ERROR]${0} can be executed only by root user"
    exit 1
fi

getent passwd ${INSTALL_USER} > /dev/null
if [ $? -ne 0 ];then
    echo "[ERROR]Invalid installation target user \"${INSTALL_USER}\""
    exit 1
fi

# systemd service file
cp ${PATH_RESOURCES_DIR}/systemd/*.service /etc/systemd/system/.
chown root:root /etc/systemd/system/platform_flight_software.service
chmod 664 /etc/systemd/system/platform_flight_software.service

systemctl daemon-reload

# scripts
mkdir -p ${PATH_APPS} || true
cp -r -f ${PATH_RESOURCES_DIR}/minimal_telemetry_publisher \
         ${PATH_RESOURCES_DIR}/management \
         ${PATH_APPS}/.
chmod +x ${PATH_APPS}/minimal_telemetry_publisher/*.py \
         ${PATH_APPS}/management/*.sh
chown -R ${INSTALL_USER}:${INSTALL_USER} ${PATH_APPS}

