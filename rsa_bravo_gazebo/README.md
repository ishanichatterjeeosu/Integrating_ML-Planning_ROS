# rsa_bravo_gazebo

Gazebo configuration for Bravo arm.

## Details

This repo contains its own Bravo URDF because Gazebo requires numerous changes from the version in `rsa_bravo_description_real`. These changes are:
* Add `gazebo_control` plugin
* Add physics parameters to fingers
* For every link:
    * Add inertial element
    * Disable gravity
    * Add friction
* For every joint:
    * Add `hardware_interface/EffortJointInterface` transmission
    * Add damping

