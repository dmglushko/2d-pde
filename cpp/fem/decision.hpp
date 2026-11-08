#pragma once
// Explained decisions (docs/vision.md, "Decisions are explained").
//
// A Decision records one choice that shaped a solve: what was decided, the
// value chosen, what the choice was based on and how well that basis is
// established. Results carry them (SolverReport::decisions,
// EvolutionMetadata::decisions) and write_report() prints them, so automatic
// choices added later are inspectable in the same place as user choices.
//
// Today every decision is the user's explicit choice: source User, basis
// "user-selected (<option>)" and evidence Unknown, because nothing checked
// that the choice suits the problem. An automatic chooser would use source
// Automatic, state its basis (e.g. "operator proven symmetric") and the
// evidence of that basis.
#include "fem/evidence.hpp"
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
enum class DecisionSource { User, Automatic };

inline const char *decision_source_name(DecisionSource source) {
  switch (source) {
  case DecisionSource::User:
    return "User";
  case DecisionSource::Automatic:
    return "Automatic";
  }
  return "Invalid";
}

struct Decision {
  std::string subject; // what was decided, e.g. "linear solver"
  std::string choice;  // the chosen value, e.g. "SparseLU"
  std::string basis;   // why, e.g. "user-selected (SolverOptions::method)"
  // How well the basis supports the choice for this problem.
  Evidence evidence = Evidence::Unknown;
  DecisionSource source = DecisionSource::User;
};

// A choice taken from the user's options. `option` names the field it came
// from, so the record says where to override it.
inline Decision user_decision(std::string subject, std::string choice,
                              const std::string &option) {
  return {std::move(subject), std::move(choice),
          "user-selected (" + option + ")", Evidence::Unknown,
          DecisionSource::User};
}
} // namespace pde2d::fem
