# teleop_camera_bringup

Three Gemini 435Le instances are launched by explicit serial number as
`cam0`, `cam1`, and `cam2`. The deployment YAML also assigns a unique physical
semantic to every serial for dataset provenance.

The bundled Orbbec SDK is not built or launched by default. Review the EULA in
`../orbbec_camera/SDK/End User License Agreement.txt`; build and runtime each
have a separate explicit acceptance gate.
