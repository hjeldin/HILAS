#pragma once

/**********************************************************
 * This file is generated by 20-sim C++ Code Generator
 *
 *  file:  include\ArmPoseControllerModel.hpp
 *  subm:  ArmPoseControllerModel
 *  model: motion_stack
 *  expmt: motion_stack
 *  date:  August 6, 2012
 *  time:  10:18:08 am
 *  user:  Campuslicentie
 *  from:  Universiteit Twente
 *  build: 4.1.2.4
 *
 **********************************************************/

/* This file describes the model functions
 that are supplied for computation.

 The model itself is the ArmPoseControllerModel.cpp file
 */

/* 20-sim include files */
#include "xxfuncs.h"
#include "xxmatrix.h"
#include "xxmodel.h"
#include "xxinteg.h"

#include <string>
#include <vector>

#include "configuration/XXModelConfiguration.hpp"

namespace motion_stack
{
	using namespace common20sim;

	class ArmPoseControllerModel: virtual Submodel20sim
	{
	public:

		/**
		 * ArmPoseControllerModel constructor
		 */
		ArmPoseControllerModel();

		/**
		 * ArmPoseControllerModel destructor
		 */
		virtual ~ArmPoseControllerModel(void);

		/**
		 * @brief Period of one computation step.
		 */
		virtual bool setPeriod(double seconds);

    /**
     * @brief Period of one computation step.
     */
    virtual double getPeriod();

		/**
		 * @brief Current time, according to the computation.
		 */
    virtual XXDouble getTime(void);

    /**
     * @brief Loads the model settings from the xml file.
     * @param uri The location of the settings file.
     */
    virtual bool loadModelConfiguration(std::string uri);

    XXModelConfiguration& getModelConfiguration();

    /**
     * @brief Configures the model.
     * @note Call after the model settings have been retrieved (with loadModelSettings).
     */
    virtual bool configure();

    /**
     * @brief Calculates the first computation step.
     * CopyInputsToVariables -> start -> CopyVariablesToOutputs -> start
     */
    virtual void start();

    /**
     * @brief Performs one calculation step.
     * CopyInputsToVariables -> step -> CopyVariablesToOutputs -> repeat
     */
    virtual void step();

    /**
     * @brief Calculates the final computation step.
     * CopyInputsToVariables -> stop -> CopyVariablesToOutputs -> nothing
     */
    virtual void stop();

    /**
     * CopyInputsToVariables
     * This private function copies the input variables from the input vector
     * @param u This is the array with all input signals for this submodel
     */
    virtual void CopyInputsToVariables () = 0;

    /**
     * CopyVariablesToOutputs
     * This private function copies the output variables to the output vector
     * @param y This is the array with all output signals from this submodel
     */
    virtual void CopyVariablesToOutputs () = 0;

    virtual int event_Initialize() = 0;

    virtual int event_edge(std::string const event_str, int inputs, double *outarr, int outputs, int major) = 0;

    virtual int event_level(std::string const event_str, int inputs, double *outarr, int outputs, int major) = 0;

    virtual int event_Terminate() = 0;

	protected:
		/**
		 * CalculateDynamic()
		 * This function calculates the dynamic equations of the model.
		 * These equations are called from the integration method
		 * to calculate the new model rates (that are then integrated).
		 */
		inline void CalculateDynamic (void);

	private:
		/* internal submodel computation methods */

		/**
		 * CalculateInitial()
		 * This function calculates the initial equations of the model.
		 * These equations are calculated before anything else
		 */
		inline void CalculateInitial (void);

		/**
		 * CalculateStatic()
		 * This function calculates the static equations of the model.
		 * These equations are only dependent from parameters and constants
		 */
		inline void CalculateStatic (void);

		/**
		 * CalculateInput()
		 * This function calculates the input equations of the model.
		 * These equations are dynamic equations that must not change
		 * in calls from the integration method (like random and delay).
		 */
		inline void CalculateInput (void);

		/**
		 * CalculateOutput()
		 * This function calculates the output equations of the model.
		 * These equations are not needed for calculation of the rates
		 * and are kept separate to make the dynamic set of equations smaller.
		 * These dynamic equations are called often more than one time for each
		 * integration step that is taken. This makes model computation much faster.
		 */
		inline void CalculateOutput (void);

		/**
		 * CalculateFinal()
		 * This function calculates the final equations of the model.
		 * These equations are calculated after all the calculations
		 * are performed
		 */
		inline void CalculateFinal (void);

		Discrete myintegmethod; ///< pointer to the integration method for this submodel

		XXModelConfiguration m_model_config;

	private:
		void setupComputation();

	};

}

