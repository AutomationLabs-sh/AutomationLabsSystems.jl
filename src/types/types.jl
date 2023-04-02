# Copyright (c) 2022: Pierre Blaud and contributors
########################################################
# This Source Code Form is subject to the terms of the #
# Mozilla Public License, v. 2.0. If a copy of the MPL #
# was not distributed with this file,  				   #
# You can obtain one at https://mozilla.org/MPL/2.0/.  #
########################################################

"""
AbstractModel
An abstract type that should be subtyped for model extensions (linear or non linear)
"""
abstract type AbstractModel end

"""
    ContinuousLinearModel
Model linear implementation with AutomationLabs.

** Fields **
* `A`: state matrix.
* `B`: input matrix.
"""
struct ContinuousLinearModel <: AbstractModel
    A::Union{Matrix,Vector}
    B::Union{Matrix,Vector}
    nbr_state::Int
    nbr_input::Int
end

"""
    DiscreteLinearModel
Model linear implementation with AutomationLabs.

** Fields **
* `A`: state matrix.
* `B`: input matrix.
"""
struct DiscreteLinearModel <: AbstractModel
    A::Union{Matrix,Vector}
    B::Union{Matrix,Vector}
    nbr_state::Int
    nbr_input::Int
end

"""
    ContinuousNonLinearModel
Model non linear implementation with AutoamtionLabs.

** Fields **
* `f`: the non linear model.
* `nbr_state`: the state number.
* `nbr_input`: the input number
"""
struct ContinuousNonLinearModel <: AbstractModel
    f::Union{Function,Flux.Chain,Flux.Parallel}
    nbr_state::Int
    nbr_input::Int
end

"""
    DiscreteNonLinearModel
Model non linear implementation with AutoamtionLabs.

** Fields **
* `f`: the non linear model.
* `nbr_state`: the state number.
* `nbr_input`: the input number
"""
struct DiscreteNonLinearModel <: AbstractModel
    f::Union{Function,Flux.Chain,Flux.Parallel}
    nbr_state::Int
    nbr_input::Int
end

#This is a type reformulation to avoid AutomationLabsIdentification types deps
"""
    Fnn
An feedforward neural network architecture type for dynamical system identification problem [ref].
"""
struct Fnn <: AbstractModel end

"""
    Rbf
An radial basis neural network architecture type for dynamical system identification problem [ref].
"""
struct Rbf <: AbstractModel end

"""
    Icnn
An input convex neural network architecture type for dynamical system identification problem [ref].
"""
struct Icnn <: AbstractModel end

"""
    ResNet
An residual layer network architecture type for dynamical system identification problem [ref].
"""
struct ResNet <: AbstractModel end

"""
    PolyNet
An poly-inception network architecture type for dynamical system identification problem [ref].
"""
struct PolyNet <: AbstractModel end

"""
    DenseNet
An densely connected network architecture type for dynamical system identification problem [ref].
"""
struct DenseNet <: AbstractModel end

"""
NeuralODE
An neural neural network ODE architecture type for dynamical system identification problem [ref].
"""
struct NeuralODE <: AbstractModel end

"""
    Rknn1
A runge-kutta neural neural network 1 architecture type for dynamical system identification problem [ref].
"""
struct Rknn1 <: AbstractModel end

"""
    Rknn2
A runge-kutta neural neural network 2 architecture type for dynamical system identification problem [ref].
"""
struct Rknn2 <: AbstractModel end

"""
    Rknn4
A runge-kutta neural neural network 4 architecture type for dynamical system identification problem [ref].
"""
struct Rknn4 <: AbstractModel end

"""
    linear
An linear (Wv --> Ax + Bu) architecture type for dynamical system identification problem [ref].
"""
struct Linear <: AbstractModel end

"""
    Rnn
A recurrent neural network architecture type for dynamical system identification problem [ref].
"""
struct Rnn <: AbstractModel end

"""
    lstm
A long short-term memory recurrent neural network architecture type for dynamical system identification problem [ref].
"""
struct Lstm <: AbstractModel end

"""
    gru
A gated recurrent unit recurrent neural network architecture type for dynamical system identification problem [ref].
"""
struct Gru <: AbstractModel end