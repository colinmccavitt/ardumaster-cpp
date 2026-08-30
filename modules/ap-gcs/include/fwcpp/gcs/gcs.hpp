#pragma once

// CPP-087 slice 2: GCS_MAVLink surface. MAVLink 2 framing, HEARTBEAT,
// COMMAND_LONG ARM/DISARM + DO_SET_MODE + COMMAND_ACK. No singleton.
// See leftover.hpp for remaining work.

#include <fwcpp/gcs/command.hpp>
#include <fwcpp/gcs/dispatch.hpp>
#include <fwcpp/gcs/framing.hpp>
#include <fwcpp/gcs/heartbeat.hpp>
#include <fwcpp/gcs/leftover.hpp>
