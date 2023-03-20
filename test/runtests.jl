# Copyright (c) 2022: Pierre Blaud and contributors
########################################################
# This Source Code Form is subject to the terms of the #
# Mozilla Public License, v. 2.0. If a copy of the MPL #
# was not distributed with this file,  				   #
# You can obtain one at https://mozilla.org/MPL/2.0/.  #
########################################################

print("Testing AutomationLabsSystems sub functions...")
took_seconds = @elapsed include("./systems_test.jl");
println("done (took ", took_seconds, " seconds)")

print("Testing AutomationLabsSystems sub functions...")
took_seconds = @elapsed include("./system_linearization_test.jl");
println("done (took ", took_seconds, " seconds)")

print("Testing AutomationLabsSystems sub functions...")
took_seconds = @elapsed include("./system_discretization_test.jl");
println("done (took ", took_seconds, " seconds)")

print("Testing AutomationLabsSystems sub functions...")
took_seconds = @elapsed include("./system_evaluation_test.jl");
println("done (took ", took_seconds, " seconds)")
