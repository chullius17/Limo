#!/usr/bin/env bash
# Source in each ROS terminal before starting nodes over the direct LIMO cable.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf 'Run this script with source so its settings remain in your terminal.\n' >&2
    exit 2
fi

_limo_wired_dds_directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../config" && pwd)" || return 1
_limo_wired_dds_profile="${_limo_wired_dds_directory}/fastdds_wired.xml"
if [[ ! -r "${_limo_wired_dds_profile}" ]]; then
    printf 'Missing Fast DDS profile: %s\n' "${_limo_wired_dds_profile}" >&2
    unset _limo_wired_dds_directory _limo_wired_dds_profile
    return 1
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${_limo_wired_dds_profile}"
export ROS_LOCALHOST_ONLY=0
printf 'ROS over Ethernet 10.74.48.1/10.74.48.2 + loopback/SHM: %s\n' "${FASTRTPS_DEFAULT_PROFILES_FILE}"
unset _limo_wired_dds_directory _limo_wired_dds_profile
