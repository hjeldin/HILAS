/**********************************************************
 * This file is generated by 20-sim C++ Code Generator
 *
 *  file:  src\SimpleEnergySupplyModel.cpp
 *  subm:  SimpleEnergySupplyModel
 *  model: motion_stack
 *  expmt: motion_stack
 *  date:  November 2, 2012
 *  time:  9:06:23 am
 *  user:  Campuslicentie
 *  from:  Universiteit Twente
 *  build: 4.1.4.1
 **********************************************************/

/* Standard include files */
#include <stdio.h>
#include <math.h>
#include <stdexcept>

/* 20-sim include files */
#include "SimpleEnergySupplyModel.hpp"

/* Orocos include */
#include <boost/algorithm/string.hpp>

using namespace std;

namespace motion_stack
{

	SimpleEnergySupplyModel::SimpleEnergySupplyModel(): m_model_config(this)
	{
		using namespace boost;

		setupComputation();
	}

	SimpleEnergySupplyModel::~SimpleEnergySupplyModel(void)
	{
		/* free memory */
		delete[] C;
		delete[] P;
		delete[] I;
		delete[] V;
		delete[] s;
		delete[] R;
		delete[] M;
		delete[] U;
		delete[] workarray;
	}

  void SimpleEnergySupplyModel::setupComputation()
  {
    start_time = 0.0;
    finish_time = 0;
    step_size = 0.001;
    time = 0;
    major = true;

    number_constants = 0;
    number_parameters = 7;
    number_initialvalues = 1;
    number_variables = 17;
    number_states = 1;
    number_rates = 1;
    number_matrices = 4;
    number_unnamed = 0;

    /* the variable arrays */
    C = new XXDouble[0 + 1]; /* constants */
    P = new XXDouble[7 + 1]; /* parameters, currently only one type of parameter exists: double */
    I = new XXDouble[1 + 1]; /* initial values */
    V = new XXDouble[17 + 1]; /* variables */

    s = new XXDouble[1 + 1]; /* states */
    R = new XXDouble[1 + 1]; /* rates (or new states) */
    M = new XXMatrix[4 + 1]; /* matrices */
    U = new XXDouble[0 + 1]; /* unnamed */
    workarray = new XXDouble[0 + 1];
  }

	bool SimpleEnergySupplyModel::loadModelConfiguration(std::string uri)
	{
	  m_model_config.load(uri);
	  return true;
	}

	XXModelConfiguration& SimpleEnergySupplyModel::getModelConfiguration()
	{
	  return m_model_config;
	}

  bool SimpleEnergySupplyModel::configure()
  {
    myintegmethod.Initialize(this);

    /* initialization phase (allocating memory) */
    initialize = true;
    //CONSTANTS
    

    //PARAMETERS
    	P[0] = 1000.0;		/* EnergyEncoding1\roundUp */
	P[1] = 100.0;		/* EnergyEncoding1\MaxCounter */
	P[2] = 10.0;		/* EnergyTankBlock\InitialEnegyState {J} */
	P[3] = 10.0;		/* EnergyTankBlock\supply */
	P[4] = 0.01;		/* EnergyTankBlock\rate */
	P[5] = 0.0;		/* SETP\CommunicationTreshold {J} */
	P[6] = 0.01;		/* SETP\SendoutPercent */


    //INITIAL VALUES
    	I[0] = 0.0;		/* EnergyEncoding1\outputCount_previous_initial */


    //MATRICES
    	M[0].mat = &V[0];		/* EnergyEncoding1\inputMessage */
	M[0].rows = 2;
	M[0].columns = 1;
	M[1].mat = &V[3];		/* EnergyEncoding1\outputMessage */
	M[1].rows = 2;
	M[1].columns = 1;
	M[2].mat = &V[12];		/* input_energy */
	M[2].rows = 2;
	M[2].columns = 1;
	M[3].mat = &V[14];		/* output_energy */
	M[3].rows = 2;
	M[3].columns = 1;


    //INITIALIZE_DEPSTATES
    //INITIALIZE_ALGLOOPS
    //INITIALIZE_CONSTRAINTS%

    //INPUTS
    //INITIALIZE_INPUTS% -> not the actual inputs

    //OUTPUTS
    //INITIALIZE_OUTPUTS% -> not the actual outputs

    //INITIALIZE_FAVORITE_PARS
    //INITIALIZE_FAVORITE_VARS

    //INITIALIZE_CONSTANTS%
    /* set the states */
    //INITIALIZE_STATES%

    /* set the matrices */
    //INITIALIZE_MATRICES%

    // overload INITIALIZE_* with values from xml
    std::vector<XVMatrix>& pps = m_model_config.getConfiguration();

    for(unsigned int i = 0; i < pps.size(); ++i)
    {
      if( static_cast<unsigned int>(pps[i].storage.rows * pps[i].storage.columns) != pps[i].values.size())
        throw std::out_of_range("" + pps[i].name);

      // Copy to XXData -> double*
      memcpy(pps[i].storage.mat, pps[i].values.data(), pps[i].values.size()*sizeof(double));
    }

    //STATES - do NOT move this line up!
    	s[0] = I[0];		/* EnergyEncoding1\outputCount_previous */


    /* end of initialization phase */
    initialize = false;
    return initialize;
  }

  void SimpleEnergySupplyModel::start()
  {
    /* calculate initial and static equations */
    CalculateInitial ();
    CalculateStatic ();
    CopyInputsToVariables ();
    CalculateInput ();
    CalculateDynamic();
    CalculateOutput ();
    CopyVariablesToOutputs ();
  }

  void SimpleEnergySupplyModel::step()
  {
    /* another precessor submodel could determine the parameters of this submodel
         and therefore the static parameter calculations need to be performed. */
    CalculateStatic ();

    /* main calculation of the model */
    CopyInputsToVariables (); //get input from port
    CalculateInput ();
    myintegmethod.Step();
    CalculateOutput ();
    CopyVariablesToOutputs (); //send output to port
  }

  void SimpleEnergySupplyModel::stop()
  {
    CopyInputsToVariables();
    /* calculate the final model equations */
    CalculateFinal ();
    CopyVariablesToOutputs();
  }

  XXDouble SimpleEnergySupplyModel::getTime(void)
  {
    return time;
  }

	/* This function calculates the initial equations of the model.
	 * These equations are calculated before anything else
	 */
	inline void SimpleEnergySupplyModel::CalculateInitial (void)
	{
				/* EnergyEncoding1\inputSum = 0; */
		V[5] = 0.0;

		/* EnergyEncoding1\outputSum = 0; */
		V[7] = 0.0;

		/* EnergyTankBlock\EnergyState = EnergyTankBlock\InitialEnegyState; */
		V[9] = P[2];

		/* SETP\outputPackage = 0; */
		V[11] = 0.0;

		/* SETP\energyQuanta = 0; */
		V[10] = 0.0;

	}

	/* This function calculates the static equations of the model.
	 * These equations are only dependent from parameters and constants
	 */
	inline void SimpleEnergySupplyModel::CalculateStatic (void)
	{
		
	}

	/* This function calculates the input equations of the model.
	 * These equations are dynamic equations that must not change
	 * in calls from the integration method (like random and delay).
	 */
	inline void SimpleEnergySupplyModel::CalculateInput (void)
	{
		
	}

	/* This function calculates the dynamic equations of the model.
	 * These equations are called from the integration method
	 * to calculate the new model rates (that are then integrated).
	 */
	inline void SimpleEnergySupplyModel::CalculateDynamic (void)
	{
			/* EnergyEncoding1\inputMessage = input_energy; */
	XXMatrixMov (&M[0], &M[2]);

		/* if (EnergyEncoding1\inputMessage[1] + 0.1 * EnergyEncoding1\roundUp) < EnergyEncoding1\inputSum */
		if ((M[0].mat[0] + 0.1 * P[0]) < V[5])
		{
			/* EnergyEncoding1\inputDiff = (EnergyEncoding1\roundUp + EnergyEncoding1\inputMessage[1]) - EnergyEncoding1\inputSum; */
			V[6] = (P[0] + M[0].mat[0]) - V[5];
		}
		else
		{
			/* EnergyEncoding1\inputDiff = EnergyEncoding1\inputMessage[1] - EnergyEncoding1\inputSum; */
			V[6] = M[0].mat[0] - V[5];
		}

		/* EnergyEncoding1\inputSum = (EnergyEncoding1\inputSum + EnergyEncoding1\inputDiff) mod EnergyEncoding1\roundUp; */
		V[5] = XXIntegerModulo ((V[5] + V[6]), P[0]);

		/* EnergyEncoding1\inputEnergy = EnergyEncoding1\inputDiff; */
		V[2] = V[6];

		/* EnergyEncoding1\outputCount = (EnergyEncoding1\outputCount_previous + 1) mod EnergyEncoding1\MaxCounter; */
		R[0] = XXIntegerModulo ((s[0] + 1.0), P[1]);

		/* EnergyEncoding1\outputSum = (EnergyEncoding1\outputSum + SETP\outputPackage) mod EnergyEncoding1\roundUp; */
		V[7] = XXIntegerModulo ((V[7] + V[11]), P[0]);

		/* EnergyEncoding1\outputMessage[1] = EnergyEncoding1\outputSum; */
		M[1].mat[0] = V[7];

		/* EnergyEncoding1\outputMessage[2] = EnergyEncoding1\outputCount; */
		M[1].mat[1] = R[0];

	/* output_energy = EnergyEncoding1\outputMessage; */
	XXMatrixMov (&M[3], &M[1]);

		/* EnergyTankBlock\EnergyState = EnergyTankBlock\EnergyState + SETP\energyQuanta; */
		V[9] = V[9] + V[10];

		/* EnergyTankBlock\EnergyState = (1 - EnergyTankBlock\rate) * EnergyTankBlock\EnergyState + (EnergyTankBlock\rate) * EnergyTankBlock\supply; */
		V[9] = (1.0 - P[4]) * V[9] + (P[4]) * P[3];

		/* EnergyTankBlock\energyState = EnergyTankBlock\EnergyState; */
		V[8] = V[9];

	/* EnergyTank = EnergyTankBlock\energyState; */
	V[16] = V[8];

		/* if SETP\CommunicationTreshold > EnergyTankBlock\energyState */
		if (P[5] > V[8])
		{
			/* SETP\outputPackage = 0; */
			V[11] = 0.0;

			/* SETP\energyQuanta = EnergyEncoding1\inputEnergy; */
			V[10] = V[2];
		}
		else
		{
			/* SETP\outputPackage = -(SETP\CommunicationTreshold - EnergyTankBlock\energyState) * SETP\SendoutPercent; */
			V[11] = -(P[5] - V[8]) * P[6];

			/* SETP\energyQuanta = EnergyEncoding1\inputEnergy - SETP\outputPackage; */
			V[10] = V[2] - V[11];
		}

	}

	/* This function calculates the output equations of the model.
	 * These equations are not needed for calculation of the rates
	 * and are kept separate to make the dynamic set of equations smaller.
	 * These dynamic equations are called often more than one time for each
	 * integration step that is taken. This makes model computation much faster.
	 */
	inline void SimpleEnergySupplyModel::CalculateOutput (void)
	{
		
			}

	/* This function calculates the final equations of the model.
	 * These equations are calculated after all the calculations
	 * are performed
	 */
	inline void SimpleEnergySupplyModel::CalculateFinal (void)
	{
		
	}

	bool SimpleEnergySupplyModel::setPeriod(double seconds)
	{
			step_size = seconds;
			return true;
	}

	double SimpleEnergySupplyModel::getPeriod()
  {
      return step_size;
  }

}

