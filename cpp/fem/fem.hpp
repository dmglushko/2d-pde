#pragma once
// Complete optional scalar FEM pipeline. Geometry-only clients need not include
// this header or link SymEngine/Eigen. See cpp/fem/README.md for the module
// map.
#include "expression.hpp"
#include "problem.hpp"
#include "weak_form.hpp"
#include "quadrature.hpp"
#include "p1_element.hpp"
#include "boundary_classification.hpp"
#include "assembly.hpp"
#include "interpolation.hpp"
#include "solver.hpp"
#include "time_grid.hpp"
#include "time_integration.hpp"
#include "evolution.hpp"
#include "error_norms.hpp"
#include "report.hpp"
