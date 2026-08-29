#pragma once

#include <fwcpp/tailsitter/tailsitter_transition.hpp>

namespace fwcpp::tailsitter {

/*
  Upstream Tailsitter::transition_fw_complete / transition_vtol_complete return bool
  (true = transition complete). ADR-0012: GCS messaging stays in the integrator.
 */
[[nodiscard]] inline bool transition_fw_complete_bool(const TailsitterTransition& transition,
                                                      const TransitionCompleteSample& sample) {
    return transition.transition_fw_complete(sample).has_value();
}

[[nodiscard]] inline bool transition_vtol_complete_bool(const TailsitterTransition& transition,
                                                        const TransitionCompleteSample& sample) {
    return transition.transition_vtol_complete(sample).has_value();
}

}  // namespace fwcpp::tailsitter
