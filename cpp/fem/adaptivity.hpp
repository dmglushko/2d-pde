#pragma once
// Optional adaptive drivers, as compositions of public components:
//
//   adapt(result, problem, plan, options)                   stationary
//   adapt_evolution(result, problem, initial, plan, options) windows in time
//
// Both take one AdaptOptions and return one AdaptiveHistory (one
// AdaptiveStep per solve or window). The headers:
//   adaptivity_types.hpp   step records (light; planners read them)
//   mesh_adaptation.hpp    how the marked mesh changes (MeshAdaptation,
//                          default MarkedBisection)
//   adaptivity_core.hpp    AdaptOptions, AdaptiveHistory and the shared step
//                          estimate -> mark -> plan mesh -> change mesh ->
//                          transfer -> refresh plan
//   adapt_stationary.hpp   adapt
//   adapt_evolution.hpp    adapt_evolution, adaptive_window_times
//   adaptivity_json.hpp    the history as JSON (not included here)
#include "fem/adapt_evolution.hpp"
#include "fem/adapt_stationary.hpp"
#include "fem/adaptivity_core.hpp"
#include "fem/adaptivity_types.hpp"
#include "fem/mesh_adaptation.hpp"
