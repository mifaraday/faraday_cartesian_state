#include "faraday_cartesian_state/faraday_cartesian_state.h"

//limb1-4 link1 initial pose in the base_link
const tf::Vector3 faraday_cartesian_state::limb1_link1_position(
        256.999999E-03, 128.293085E-03, 147.144575E-03);
const tf::Vector3 faraday_cartesian_state::limb1_link1_orientation(
        0.0, 0.0, -0.523598775598299);

const tf::Vector3 faraday_cartesian_state::limb2_link1_position(
        243.823085E-03, -172.53E-03, 145.659457E-03);
const tf::Vector3 faraday_cartesian_state::limb2_link1_orientation(
        0.0, 0.0, -2.094395102393196);

const tf::Vector3 faraday_cartesian_state::limb3_link1_position(
        -243.823085E-03, -172.53E-03, 145.659457E-03);
const tf::Vector3 faraday_cartesian_state::limb3_link1_orientation(
        0.0, 0.0, 2.094395102393196);

const tf::Vector3 faraday_cartesian_state::limb4_link1_position(
        -256.999999E-03, 128.293085E-03, 147.144575E-03);
const tf::Vector3 faraday_cartesian_state::limb4_link1_orientation(
        0.0, 0.0, 0.523598775598299);

//limb5 link1-3 initial orientation in the base_link
const tf::Matrix3x3 faraday_cartesian_state::limb5_link1_rotation(
        0.0, -1.0, 0.0,
        0.0,  0.0, 1.0,
       -1.0,  0.0, 0.0);
const tf::Vector3 faraday_cartesian_state::limb5_link1_position(
        -0.175, -0.069, -0.046);

const tf::Matrix3x3 faraday_cartesian_state::limb5_link2_rotation(
        0.0,  0.0,  1.0,
        1.0,  0.0,  0.0,
        0.0,  1.0,  0.0);
const tf::Vector3 faraday_cartesian_state::limb5_link2_position(
        0.0, -0.069, -0.0935);

const tf::Matrix3x3 faraday_cartesian_state::limb5_link3_rotation(
       -1.0,   0.0, 0.0,
        0.0,  -1.0, 0.0,
        0.0,   0.0, 1.0);

const double faraday_cartesian_state::limb5_link1_y=0.069;

const double faraday_cartesian_state::limb5_link2_y=0.069;

const double faraday_cartesian_state::limb5_link3_y=0.033;

const double faraday_cartesian_state::limb5_link3_z=0.234;




