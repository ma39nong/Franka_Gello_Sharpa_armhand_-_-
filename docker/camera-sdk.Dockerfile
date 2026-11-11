# The build script verifies the official archive and preserves its licenses.
FROM franka-upper-body-teleop:latest
COPY sdk/ /opt/teleop-orbbec-sdk/
COPY camera_sdk_entrypoint.sh /opt/teleop-orbbec-sdk/entrypoint.sh
ENTRYPOINT ["bash", "/opt/teleop-orbbec-sdk/entrypoint.sh"]
