# Copyright (c) 2022: Pierre Blaud and contributors
########################################################
# This Source Code Form is subject to the terms of the #
# Mozilla Public License, v. 2.0. If a copy of the MPL #
# was not distributed with this file,  				   #
# You can obtain one at https://mozilla.org/MPL/2.0/.  #
########################################################
module SystemsEvaluationTests

using Test
using AutomationLabsSystems
using AutomationLabsIdentification
using MathematicalSystems
using LazySets
using MLJ
using Flux
using MLJMultivariateStatsInterface

### Discrete ###
@testset "Model evaluation fnn, resnet, ..." begin

    fnn = machine("./models_saved/fnn_train_result.jls")
    icnn = machine("./models_saved/icnn_train_result.jls")
    resnet = machine("./models_saved/resnet_train_result.jls")
    polynet = machine("./models_saved/polynet_train_result.jls")
    densenet = machine("./models_saved/densenet_train_result.jls")
    rbf = machine("./models_saved/rbf_train_result.jls")
    neuralode = machine("./models_saved/NeuralODE_train_result.jls")
    rknn1 = machine("./models_saved/rknn1_train_result.jls")
    rknn2 = machine("./models_saved/rknn2_train_result.jls")
    rknn4 = machine("./models_saved/rknn4_train_result.jls")
    rnn = machine("./models_saved/rnn_train_result.jls")
    lstm = machine("./models_saved/lstm_train_result.jls")
    gru = machine("./models_saved/gru_train_result.jls")
    linear = machine("./models_saved/linear_regressor_train_result.jls")

    fnn_type = AutomationLabsIdentification.get_mlj_model_type(fnn)
    icnn_type = AutomationLabsIdentification.get_mlj_model_type(icnn)
    resnet_type = AutomationLabsIdentification.get_mlj_model_type(resnet)
    polynet_type = AutomationLabsIdentification.get_mlj_model_type(polynet)
    densenet_type = AutomationLabsIdentification.get_mlj_model_type(densenet)
    rbf_type = AutomationLabsIdentification.get_mlj_model_type(rbf)
    neuralode_type = AutomationLabsIdentification.get_mlj_model_type(neuralode)
    rknn1_type = AutomationLabsIdentification.get_mlj_model_type(rknn1)
    rknn2_type = AutomationLabsIdentification.get_mlj_model_type(rknn2)
    rknn4_type = AutomationLabsIdentification.get_mlj_model_type(rknn4)
    rnn_type = AutomationLabsIdentification.get_mlj_model_type(rnn)
    lstm_type = AutomationLabsIdentification.get_mlj_model_type(lstm)
    gru_type = AutomationLabsIdentification.get_mlj_model_type(gru)
    linear_type = AutomationLabsIdentification.get_mlj_model_type(linear)

    @test typeof(fnn_type) == typeof(AutomationLabsIdentification.Fnn())
    @test typeof(icnn_type) == typeof(AutomationLabsIdentification.Icnn())
    @test typeof(resnet_type) == typeof(AutomationLabsIdentification.ResNet())
    @test typeof(polynet_type) == typeof(AutomationLabsIdentification.PolyNet())
    @test typeof(densenet_type) == typeof(AutomationLabsIdentification.DenseNet())
    @test typeof(rbf_type) == typeof(AutomationLabsIdentification.Rbf())
    @test typeof(neuralode_type) == typeof(AutomationLabsIdentification.NeuralODE())
    @test typeof(rknn1_type) == typeof(AutomationLabsIdentification.Rknn1())
    @test typeof(rknn2_type) == typeof(AutomationLabsIdentification.Rknn2())
    @test typeof(rknn4_type) == typeof(AutomationLabsIdentification.Rknn4())
    @test typeof(rnn_type) == typeof(AutomationLabsIdentification.Rnn())
    @test typeof(lstm_type) == typeof(AutomationLabsIdentification.Lstm())
    @test typeof(gru_type) == typeof(AutomationLabsIdentification.Gru())
    @test typeof(linear_type) ==
          typeof(MLJMultivariateStatsInterface.MultitargetLinearRegressor(bias = false))

    f_fnn = AutomationLabsIdentification.extract_model_from_machine(fnn_type, fnn)
    f_icnn = AutomationLabsIdentification.extract_model_from_machine(icnn_type, icnn)
    f_resnet = AutomationLabsIdentification.extract_model_from_machine(resnet_type, resnet)
    f_polynet = AutomationLabsIdentification.extract_model_from_machine(polynet_type, polynet)
    f_densenet = AutomationLabsIdentification.extract_model_from_machine(densenet_type, densenet)
    f_rbf = AutomationLabsIdentification.extract_model_from_machine(rbf_type, rbf)
    f_neuralode = AutomationLabsIdentification.extract_model_from_machine(neuralode_type, neuralode)
    f_rknn1 = AutomationLabsIdentification.extract_model_from_machine(rknn1_type, rknn1)
    f_rknn2 = AutomationLabsIdentification.extract_model_from_machine(rknn2_type, rknn2)
    f_rknn4 = AutomationLabsIdentification.extract_model_from_machine(rknn4_type, rknn4)
    f_rnn = AutomationLabsIdentification.extract_model_from_machine(rnn_type, rnn)
    f_lstm = AutomationLabsIdentification.extract_model_from_machine(lstm_type, lstm)
    f_gru = AutomationLabsIdentification.extract_model_from_machine(gru_type, gru)
    f_linear = AutomationLabsIdentification.extract_model_from_machine(linear_type, linear)

    nbr_state = 4
    nbr_input = 2
    variation = "discrete"

    sys_fnn = proceed_system(f_fnn[:f], nbr_state, nbr_input, variation)
    sys_icnn = proceed_system(f_icnn[:f], nbr_state, nbr_input, variation)
    sys_resnet = proceed_system(f_resnet[:f], nbr_state, nbr_input, variation)
    sys_polynet = proceed_system(f_polynet[:f], nbr_state, nbr_input, variation)
    sys_densenet = proceed_system(f_densenet[:f], nbr_state, nbr_input, variation)
    sys_rbf = proceed_system(f_rbf[:f], nbr_state, nbr_input, variation)
    sys_neuralode = proceed_system(f_neuralode[:f], nbr_state, nbr_input, variation)
    sys_rknn1 = proceed_system(f_rknn1[:f], nbr_state, nbr_input, variation)
    sys_rknn2 = proceed_system(f_rknn2[:f], nbr_state, nbr_input, variation)
    sys_rknn4 = proceed_system(f_rknn4[:f], nbr_state, nbr_input, variation)
    sys_rnn = proceed_system(f_rnn[:f], nbr_state, nbr_input, variation)
    sys_lstm = proceed_system(f_lstm[:f], nbr_state, nbr_input, variation)
    sys_gru = proceed_system(f_gru[:f], nbr_state, nbr_input, variation)
    sys_linear = proceed_system(f_linear[:A], f_linear[:B], nbr_state, nbr_input, variation)

    fnn_type_2 = proceed_system_model_evaluation(sys_fnn)
    icnn_type_2 = proceed_system_model_evaluation(sys_icnn)
    resnet_type_2 = proceed_system_model_evaluation(sys_resnet)
    polynet_type_2 = proceed_system_model_evaluation(sys_polynet)
    densenet_type_2 = proceed_system_model_evaluation(sys_densenet)
    rbf_type_2 = proceed_system_model_evaluation(sys_rbf)
    neuralode_type_2 = proceed_system_model_evaluation(sys_neuralode)
    rknn1_type_2 = proceed_system_model_evaluation(sys_rknn1)
    rknn2_type_2 = proceed_system_model_evaluation(sys_rknn2)
    rknn4_type_2 = proceed_system_model_evaluation(sys_rknn4)
    rnn_type_2 = proceed_system_model_evaluation(sys_rnn) 
    lstm_type_2 = proceed_system_model_evaluation(sys_lstm) 
    gru_type_2 = proceed_system_model_evaluation(sys_gru) 

    @test AutomationLabsSystems.Fnn == typeof(fnn_type_2)
    @test AutomationLabsSystems.Icnn  == typeof(icnn_type_2)
    @test AutomationLabsSystems.ResNet  == typeof(resnet_type_2)
    @test AutomationLabsSystems.PolyNet  == typeof(polynet_type_2)
    @test AutomationLabsSystems.DenseNet  == typeof(densenet_type_2)
    @test AutomationLabsSystems.Rbf == typeof(rbf_type_2)
    @test AutomationLabsSystems.NeuralODE  == typeof(neuralode_type_2)
    @test AutomationLabsSystems.Rknn1  == typeof(rknn1_type_2)
    @test AutomationLabsSystems.Rknn2  == typeof(rknn2_type_2)
    @test AutomationLabsSystems.Rknn4  == typeof(rknn4_type_2)
    @test AutomationLabsSystems.Rnn  == typeof(rnn_type_2)
    @test AutomationLabsSystems.Lstm  == typeof(lstm_type_2)
    @test AutomationLabsSystems.Gru  == typeof(gru_type_2)

end

end
