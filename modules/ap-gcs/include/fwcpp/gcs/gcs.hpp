#pragma once

// CPP-087 slice 3: GCS_MAVLink surface. MAVLink 2 framing, HEARTBEAT,
// COMMAND_LONG ARM/DISARM + DO_SET_MODE + COMMAND_ACK, PARAM_REQUEST_LIST /
// PARAM_SET / PARAM_VALUE. No singleton. See leftover.hpp for remaining work.

#include <fwcpp/gcs/command.hpp>
#include <fwcpp/gcs/dispatch.hpp>
#include <fwcpp/gcs/framing.hpp>
#include <fwcpp/gcs/heartbeat.hpp>
#include <fwcpp/gcs/leftover.hpp>
#include <fwcpp/gcs/param.hpp>
