#pragma once

#include "ParamBank.hpp"
#include "utilities.hpp"

namespace lem_dynamics_sim_{
    
    State derative_tire_model( const ParamBank& P, const State& x, const Input& u) ;

}
