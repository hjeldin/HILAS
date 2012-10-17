/**********************************************************
 * This file is generated by 20-sim C++ Code Generator
 *
 *  file:  src\CartesianControllerModel.cpp
 *  subm:  CartesianControllerModel
 *  model: motion_stack
 *  expmt: motion_stack
 *  date:  October 17, 2012
 *  time:  12:46:40 pm
 *  user:  Campuslicentie
 *  from:  Universiteit Twente
 *  build: 4.1.4.1
 **********************************************************/

/* Standard include files */
#include <stdio.h>
#include <math.h>
#include <stdexcept>

/* 20-sim include files */
#include "CartesianControllerModel.hpp"

/* Orocos include */
#include <boost/algorithm/string.hpp>

using namespace std;

namespace motion_stack
{

	CartesianControllerModel::CartesianControllerModel(): m_model_config(this)
	{
		using namespace boost;

		setupComputation();
	}

	CartesianControllerModel::~CartesianControllerModel(void)
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

  void CartesianControllerModel::setupComputation()
  {
    start_time = 0.0;
    finish_time = 0;
    step_size = 0.001;
    time = 0;
    major = true;

    number_constants = 0;
    number_parameters = 1;
    number_initialvalues = 16;
    number_variables = 333;
    number_states = 16;
    number_rates = 16;
    number_matrices = 98;
    number_unnamed = 638;

    /* the variable arrays */
    C = new XXDouble[0 + 1]; /* constants */
    P = new XXDouble[1 + 1]; /* parameters, currently only one type of parameter exists: double */
    I = new XXDouble[16 + 1]; /* initial values */
    V = new XXDouble[333 + 1]; /* variables */

    s = new XXDouble[16 + 1]; /* states */
    R = new XXDouble[16 + 1]; /* rates (or new states) */
    M = new XXMatrix[98 + 1]; /* matrices */
    U = new XXDouble[638 + 1]; /* unnamed */
    workarray = new XXDouble[9 + 1];
  }

	bool CartesianControllerModel::loadModelConfiguration(std::string uri)
	{
	  m_model_config.load(uri);
	  return true;
	}

	XXModelConfiguration& CartesianControllerModel::getModelConfiguration()
	{
	  return m_model_config;
	}

  bool CartesianControllerModel::configure()
  {
    myintegmethod.Initialize(this);

    /* initialization phase (allocating memory) */
    initialize = true;
    //CONSTANTS
    

    //PARAMETERS
    	P[0] = 0.0;		/* Damping\K */


    //INITIAL VALUES
    	I[0] = 0.0;		/* Delay\output_initial */
	I[1] = 0.0;		
	I[2] = 0.0;		
	I[3] = 0.0;		
	I[4] = 0.0;		
	I[5] = 0.0;		
	I[6] = 0.0;		
	I[7] = 0.0;		
	I[8] = 0.0;		
	I[9] = 0.0;		
	I[10] = 0.0;		
	I[11] = 0.0;		
	I[12] = 0.0;		
	I[13] = 0.0;		
	I[14] = 0.0;		
	I[15] = 0.0;		


    //MATRICES
    	M[0].mat = &V[0];		/* CartesianSpaceStiffness\effort */
	M[0].rows = 6;
	M[0].columns = 1;
	M[1].mat = &V[6];		/* CartesianSpaceStiffness\Htip0 */
	M[1].rows = 4;
	M[1].columns = 4;
	M[2].mat = &V[22];		/* CartesianSpaceStiffness\cart_stiffness */
	M[2].rows = 9;
	M[2].columns = 1;
	M[3].mat = &V[31];		/* CartesianSpaceStiffness\HtipCC */
	M[3].rows = 4;
	M[3].columns = 4;
	M[4].mat = &V[47];		/* CartesianSpaceStiffness\Kt */
	M[4].rows = 3;
	M[4].columns = 1;
	M[5].mat = &V[50];		/* CartesianSpaceStiffness\Kr */
	M[5].rows = 3;
	M[5].columns = 1;
	M[6].mat = &V[53];		/* CartesianSpaceStiffness\Kc */
	M[6].rows = 3;
	M[6].columns = 1;
	M[7].mat = &V[56];		/* CartesianSpaceStiffness\Kcc */
	M[7].rows = 6;
	M[7].columns = 6;
	M[8].mat = &V[92];		/* CartesianSpaceStiffness\H1_0 */
	M[8].rows = 4;
	M[8].columns = 4;
	M[9].mat = &V[108];		/* CartesianSpaceStiffness\costiffness1 */
	M[9].rows = 3;
	M[9].columns = 3;
	M[10].mat = &V[117];		/* CartesianSpaceStiffness\costiffness2 */
	M[10].rows = 3;
	M[10].columns = 3;
	M[11].mat = &V[126];		/* CartesianSpaceStiffness\costiffness3 */
	M[11].rows = 3;
	M[11].columns = 3;
	M[12].mat = &V[135];		/* CartesianSpaceStiffness\dummy1 */
	M[12].rows = 3;
	M[12].columns = 3;
	M[13].mat = &V[144];		/* CartesianSpaceStiffness\dummy2 */
	M[13].rows = 3;
	M[13].columns = 3;
	M[14].mat = &V[153];		/* CartesianSpaceStiffness\dummy3 */
	M[14].rows = 3;
	M[14].columns = 3;
	M[15].mat = &V[162];		/* CartesianSpaceStiffness\dummy4 */
	M[15].rows = 3;
	M[15].columns = 3;
	M[16].mat = &V[171];		/* CartesianSpaceStiffness\dummy5 */
	M[16].rows = 3;
	M[16].columns = 3;
	M[17].mat = &V[180];		/* CartesianSpaceStiffness\skew21 */
	M[17].rows = 3;
	M[17].columns = 3;
	M[18].mat = &V[189];		/* CartesianSpaceStiffness\orientation21 */
	M[18].rows = 3;
	M[18].columns = 3;
	M[19].mat = &V[198];		/* Damping\output */
	M[19].rows = 6;
	M[19].columns = 1;
	M[20].mat = &V[204];		/* Damping\Damping */
	M[20].rows = 6;
	M[20].columns = 1;
	M[21].mat = &V[210];		/* dHToErrors1\Twist */
	M[21].rows = 6;
	M[21].columns = 1;
	M[22].mat = &V[216];		/* dHToErrors1\H_vp_base */
	M[22].rows = 4;
	M[22].columns = 4;
	M[23].mat = &V[234];		/* dHToErrors1\norm_omega */
	M[23].rows = 3;
	M[23].columns = 1;
	M[24].mat = &V[237];		/* dHToErrors1\omega */
	M[24].rows = 3;
	M[24].columns = 1;
	M[25].mat = &V[240];		/* PlusMinus2\output */
	M[25].rows = 6;
	M[25].columns = 1;
	M[26].mat = &V[246];		/* PlusMinus2\plus1 */
	M[26].rows = 6;
	M[26].columns = 1;
	M[27].mat = &V[252];		/* PlusMinus5\output */
	M[27].rows = 6;
	M[27].columns = 1;
	M[28].mat = &V[258];		/* CartesianDamping */
	M[28].rows = 6;
	M[28].columns = 1;
	M[29].mat = &V[264];		/* T_tooltip_00 */
	M[29].rows = 6;
	M[29].columns = 1;
	M[30].mat = &V[270];		/* Hvp0 */
	M[30].rows = 4;
	M[30].columns = 4;
	M[31].mat = &V[286];		/* Htip0 */
	M[31].rows = 4;
	M[31].columns = 4;
	M[32].mat = &V[302];		/* StiffnessCC */
	M[32].rows = 9;
	M[32].columns = 1;
	M[33].mat = &V[311];		/* HtipCC */
	M[33].rows = 4;
	M[33].columns = 4;
	M[34].mat = &V[327];		/* W_tooltip_00 */
	M[34].rows = 6;
	M[34].columns = 1;
	M[35].mat = &R[0];		/* Delay\input */
	M[35].rows = 4;
	M[35].columns = 4;
	M[36].mat = &s[0];		/* Delay\output */
	M[36].rows = 4;
	M[36].columns = 4;
	M[37].mat = &I[0];		/* Delay\output_initial */
	M[37].rows = 4;
	M[37].columns = 4;
	M[38].mat = &U[0];		/* U1 */
	M[38].rows = 3;
	M[38].columns = 3;
	M[39].mat = &U[9];		/* U2 */
	M[39].rows = 3;
	M[39].columns = 3;
	M[40].mat = &U[18];		/* U3 */
	M[40].rows = 3;
	M[40].columns = 3;
	M[41].mat = &U[27];		/* U4 */
	M[41].rows = 3;
	M[41].columns = 3;
	M[42].mat = &U[36];		/* U5 */
	M[42].rows = 3;
	M[42].columns = 3;
	M[43].mat = &U[45];		/* U6 */
	M[43].rows = 4;
	M[43].columns = 4;
	M[44].mat = &U[61];		/* U7 */
	M[44].rows = 1;
	M[44].columns = 1;
	M[45].mat = &U[62];		/* U8 */
	M[45].rows = 3;
	M[45].columns = 3;
	M[46].mat = &U[71];		/* U9 */
	M[46].rows = 1;
	M[46].columns = 1;
	M[47].mat = &U[72];		/* U10 */
	M[47].rows = 1;
	M[47].columns = 1;
	M[48].mat = &U[73];		/* U11 */
	M[48].rows = 3;
	M[48].columns = 1;
	M[49].mat = &U[76];		/* U12 */
	M[49].rows = 3;
	M[49].columns = 1;
	M[50].mat = &U[79];		/* U13 */
	M[50].rows = 3;
	M[50].columns = 1;
	M[51].mat = &U[82];		/* U14 */
	M[51].rows = 1;
	M[51].columns = 1;
	M[52].mat = &U[83];		/* U15 */
	M[52].rows = 6;
	M[52].columns = 6;
	M[53].mat = &U[119];		/* U16 */
	M[53].rows = 3;
	M[53].columns = 3;
	M[54].mat = &U[128];		/* U17 */
	M[54].rows = 3;
	M[54].columns = 3;
	M[55].mat = &U[137];		/* U18 */
	M[55].rows = 3;
	M[55].columns = 3;
	M[56].mat = &U[146];		/* U19 */
	M[56].rows = 3;
	M[56].columns = 3;
	M[57].mat = &U[155];		/* U20 */
	M[57].rows = 3;
	M[57].columns = 3;
	M[58].mat = &U[164];		/* U21 */
	M[58].rows = 3;
	M[58].columns = 3;
	M[59].mat = &U[173];		/* U22 */
	M[59].rows = 4;
	M[59].columns = 4;
	M[60].mat = &U[189];		/* U23 */
	M[60].rows = 4;
	M[60].columns = 4;
	M[61].mat = &U[205];		/* U24 */
	M[61].rows = 4;
	M[61].columns = 4;
	M[62].mat = &U[221];		/* U25 */
	M[62].rows = 4;
	M[62].columns = 4;
	M[63].mat = &U[237];		/* U26 */
	M[63].rows = 3;
	M[63].columns = 1;
	M[64].mat = &U[240];		/* U27 */
	M[64].rows = 3;
	M[64].columns = 3;
	M[65].mat = &U[249];		/* U28 */
	M[65].rows = 3;
	M[65].columns = 3;
	M[66].mat = &U[258];		/* U29 */
	M[66].rows = 3;
	M[66].columns = 3;
	M[67].mat = &U[267];		/* U30 */
	M[67].rows = 3;
	M[67].columns = 3;
	M[68].mat = &U[276];		/* U31 */
	M[68].rows = 3;
	M[68].columns = 3;
	M[69].mat = &U[285];		/* U32 */
	M[69].rows = 3;
	M[69].columns = 3;
	M[70].mat = &U[294];		/* U33 */
	M[70].rows = 3;
	M[70].columns = 3;
	M[71].mat = &U[303];		/* U34 */
	M[71].rows = 3;
	M[71].columns = 3;
	M[72].mat = &U[312];		/* U35 */
	M[72].rows = 3;
	M[72].columns = 3;
	M[73].mat = &U[321];		/* U36 */
	M[73].rows = 3;
	M[73].columns = 3;
	M[74].mat = &U[330];		/* U37 */
	M[74].rows = 3;
	M[74].columns = 3;
	M[75].mat = &U[339];		/* U38 */
	M[75].rows = 3;
	M[75].columns = 3;
	M[76].mat = &U[348];		/* U39 */
	M[76].rows = 3;
	M[76].columns = 3;
	M[77].mat = &U[357];		/* U40 */
	M[77].rows = 3;
	M[77].columns = 3;
	M[78].mat = &U[366];		/* U41 */
	M[78].rows = 3;
	M[78].columns = 3;
	M[79].mat = &U[375];		/* U42 */
	M[79].rows = 3;
	M[79].columns = 3;
	M[80].mat = &U[384];		/* U43 */
	M[80].rows = 3;
	M[80].columns = 3;
	M[81].mat = &U[393];		/* U44 */
	M[81].rows = 3;
	M[81].columns = 3;
	M[82].mat = &U[402];		/* U45 */
	M[82].rows = 3;
	M[82].columns = 3;
	M[83].mat = &U[411];		/* U46 */
	M[83].rows = 3;
	M[83].columns = 3;
	M[84].mat = &U[420];		/* U47 */
	M[84].rows = 3;
	M[84].columns = 3;
	M[85].mat = &U[429];		/* U48 */
	M[85].rows = 3;
	M[85].columns = 3;
	M[86].mat = &U[438];		/* U49 */
	M[86].rows = 3;
	M[86].columns = 3;
	M[87].mat = &U[447];		/* U50 */
	M[87].rows = 3;
	M[87].columns = 3;
	M[88].mat = &U[456];		/* U51 */
	M[88].rows = 3;
	M[88].columns = 3;
	M[89].mat = &U[465];		/* U52 */
	M[89].rows = 3;
	M[89].columns = 3;
	M[90].mat = &U[474];		/* U53 */
	M[90].rows = 3;
	M[90].columns = 3;
	M[91].mat = &U[483];		/* U54 */
	M[91].rows = 3;
	M[91].columns = 3;
	M[92].mat = &U[492];		/* U55 */
	M[92].rows = 6;
	M[92].columns = 6;
	M[93].mat = &U[528];		/* U56 */
	M[93].rows = 6;
	M[93].columns = 6;
	M[94].mat = &U[564];		/* U57 */
	M[94].rows = 6;
	M[94].columns = 6;
	M[95].mat = &U[600];		/* U58 */
	M[95].rows = 4;
	M[95].columns = 4;
	M[96].mat = &U[616];		/* U59 */
	M[96].rows = 4;
	M[96].columns = 4;
	M[97].mat = &U[632];		/* U60 */
	M[97].rows = 6;
	M[97].columns = 1;


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
    	s[0] = I[0];		/* Delay\output */
	s[1] = I[1];
	s[2] = I[2];
	s[3] = I[3];
	s[4] = I[4];
	s[5] = I[5];
	s[6] = I[6];
	s[7] = I[7];
	s[8] = I[8];
	s[9] = I[9];
	s[10] = I[10];
	s[11] = I[11];
	s[12] = I[12];
	s[13] = I[13];
	s[14] = I[14];
	s[15] = I[15];


    /* end of initialization phase */
    initialize = false;
    return initialize;
  }

  void CartesianControllerModel::start()
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

  void CartesianControllerModel::step()
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

  void CartesianControllerModel::stop()
  {
    CopyInputsToVariables();
    /* calculate the final model equations */
    CalculateFinal ();
    CopyVariablesToOutputs();
  }

  XXDouble CartesianControllerModel::getTime(void)
  {
    return time;
  }

	/* This function calculates the initial equations of the model.
	 * These equations are calculated before anything else
	 */
	inline void CartesianControllerModel::CalculateInitial (void)
	{
				/* CartesianSpaceStiffness\effort = 0; */
		XXMatrixScalarMov (&M[0], 0.0);

	}

	/* This function calculates the static equations of the model.
	 * These equations are only dependent from parameters and constants
	 */
	inline void CartesianControllerModel::CalculateStatic (void)
	{
		
	}

	/* This function calculates the input equations of the model.
	 * These equations are dynamic equations that must not change
	 * in calls from the integration method (like random and delay).
	 */
	inline void CartesianControllerModel::CalculateInput (void)
	{
		
	}

	/* This function calculates the dynamic equations of the model.
	 * These equations are called from the integration method
	 * to calculate the new model rates (that are then integrated).
	 */
	inline void CartesianControllerModel::CalculateDynamic (void)
	{
			/* Damping\Damping = CartesianDamping; */
	XXMatrixMov (&M[20], &M[28]);

	/* PlusMinus2\plus1 = T_tooltip_00; */
	XXMatrixMov (&M[26], &M[29]);

	/* Delay\input = Hvp0; */
	XXMatrixMov (&M[35], &M[30]);

	/* CartesianSpaceStiffness\cart_stiffness = StiffnessCC; */
	XXMatrixMov (&M[2], &M[32]);

	/* CartesianSpaceStiffness\Kt = CartesianSpaceStiffness\cart_stiffness[1:3]; */
	M[4].mat[0] = M[2].mat[0];
	M[4].mat[1] = M[2].mat[1];
	M[4].mat[2] = M[2].mat[2];

	/* CartesianSpaceStiffness\Kr = CartesianSpaceStiffness\cart_stiffness[4:6]; */
	M[5].mat[0] = M[2].mat[3];
	M[5].mat[1] = M[2].mat[4];
	M[5].mat[2] = M[2].mat[5];

	/* CartesianSpaceStiffness\Kc = CartesianSpaceStiffness\cart_stiffness[7:9]; */
	M[6].mat[0] = M[2].mat[6];
	M[6].mat[1] = M[2].mat[7];
	M[6].mat[2] = M[2].mat[8];

	/* CartesianSpaceStiffness\Kcc[1:3,1:3] = diag (CartesianSpaceStiffness\Kr); */
	XXMatrixDiag (&M[38], &M[5]);
	M[7].mat[0] = M[38].mat[0];
	M[7].mat[1] = M[38].mat[1];
	M[7].mat[2] = M[38].mat[2];
	M[7].mat[6] = M[38].mat[3];
	M[7].mat[7] = M[38].mat[4];
	M[7].mat[8] = M[38].mat[5];
	M[7].mat[12] = M[38].mat[6];
	M[7].mat[13] = M[38].mat[7];
	M[7].mat[14] = M[38].mat[8];

	/* CartesianSpaceStiffness\Kcc[1:3,4:6] = diag (CartesianSpaceStiffness\Kc); */
	XXMatrixDiag (&M[39], &M[6]);
	M[7].mat[3] = M[39].mat[0];
	M[7].mat[4] = M[39].mat[1];
	M[7].mat[5] = M[39].mat[2];
	M[7].mat[9] = M[39].mat[3];
	M[7].mat[10] = M[39].mat[4];
	M[7].mat[11] = M[39].mat[5];
	M[7].mat[15] = M[39].mat[6];
	M[7].mat[16] = M[39].mat[7];
	M[7].mat[17] = M[39].mat[8];

	/* CartesianSpaceStiffness\Kcc[4:6,1:3] = transpose (diag (CartesianSpaceStiffness\Kc)); */
	XXMatrixDiag (&M[41], &M[6]);
	XXMatrixTranspose (&M[40], &M[41]);
	M[7].mat[18] = M[40].mat[0];
	M[7].mat[19] = M[40].mat[1];
	M[7].mat[20] = M[40].mat[2];
	M[7].mat[24] = M[40].mat[3];
	M[7].mat[25] = M[40].mat[4];
	M[7].mat[26] = M[40].mat[5];
	M[7].mat[30] = M[40].mat[6];
	M[7].mat[31] = M[40].mat[7];
	M[7].mat[32] = M[40].mat[8];

	/* CartesianSpaceStiffness\Kcc[4:6,4:6] = diag (CartesianSpaceStiffness\Kt); */
	XXMatrixDiag (&M[42], &M[4]);
	M[7].mat[21] = M[42].mat[0];
	M[7].mat[22] = M[42].mat[1];
	M[7].mat[23] = M[42].mat[2];
	M[7].mat[27] = M[42].mat[3];
	M[7].mat[28] = M[42].mat[4];
	M[7].mat[29] = M[42].mat[5];
	M[7].mat[33] = M[42].mat[6];
	M[7].mat[34] = M[42].mat[7];
	M[7].mat[35] = M[42].mat[8];

		/* dHToErrors1\H_vp_base = inverseH (Delay\input) * Delay\output; */
		XXMatrixInverseH (&M[43], &M[35]);
		XXMatrixMul (&M[22], &M[43], &M[36]);

		/* dHToErrors1\aldo = (trace (dHToErrors1\H_vp_base[1:3,1:3]) - 1) / 2; */
		M[45].mat[0] = M[22].mat[0];
		M[45].mat[1] = M[22].mat[1];
		M[45].mat[2] = M[22].mat[2];
		M[45].mat[3] = M[22].mat[4];
		M[45].mat[4] = M[22].mat[5];
		M[45].mat[5] = M[22].mat[6];
		M[45].mat[6] = M[22].mat[8];
		M[45].mat[7] = M[22].mat[9];
		M[45].mat[8] = M[22].mat[10];
		M[44].mat[0] = XXMatrixTrace (&M[45]);
		V[232] = (XXMatrixTrace (&M[45]) - 1.0) / 2.0;

		/* if (dHToErrors1\aldo < -1) */
		if (V[232] < -1.0)
		{
			/* dHToErrors1\aldo = -1; */
			V[232] = -1.0;
		}

		/* if (dHToErrors1\aldo > 1) */
		if (V[232] > 1.0)
		{
			/* dHToErrors1\aldo = 1; */
			V[232] = 1.0;
		}

		/* dHToErrors1\theta = arccos (dHToErrors1\aldo); */
		V[233] = acos (V[232]);

		/* if (dHToErrors1\theta == 0) */
		if (V[233] == 0.0)
		{
			/* dHToErrors1\norm_omega = 0; */
			XXMatrixScalarMov (&M[23], 0.0);
		}
		else
		{
			/* dHToErrors1\norm_omega = (1 / (2 * sin (dHToErrors1\theta))) * [dHToErrors1\H_vp_base[3,2] - dHToErrors1\H_vp_base[2,3]; dHToErrors1\H_vp_base[1,3] - dHToErrors1\H_vp_base[3,1]; dHToErrors1\H_vp_base[2,1] - dHToErrors1\H_vp_base[1,2]]; */
			M[48].mat[0] = M[22].mat[9] - M[22].mat[6];
			M[48].mat[1] = M[22].mat[2] - M[22].mat[8];
			M[48].mat[2] = M[22].mat[4] - M[22].mat[1];
			XXScalarMatrixMul (&M[23], (1.0 / (2.0 * sin (V[233]))), &M[48]);
		}

		/* dHToErrors1\omega = dHToErrors1\theta * dHToErrors1\norm_omega; */
		XXScalarMatrixMul (&M[24], V[233], &M[23]);

		/* dHToErrors1\Twist[1:3] = -dHToErrors1\H_vp_base[1:3,4]; */
		M[50].mat[0] = M[22].mat[3];
		M[50].mat[1] = M[22].mat[7];
		M[50].mat[2] = M[22].mat[11];
		XXMatrixInv (&M[49], &M[50]);
		M[21].mat[0] = M[49].mat[0];
		M[21].mat[1] = M[49].mat[1];
		M[21].mat[2] = M[49].mat[2];

		/* dHToErrors1\Twist[4:6] = -dHToErrors1\omega[3]; */
		M[51].mat[0] = -M[24].mat[2];
		M[21].mat[3] = -M[24].mat[2];
		M[21].mat[4] = -M[24].mat[2];
		M[21].mat[5] = -M[24].mat[2];

	/* PlusMinus2\output = PlusMinus2\plus1 - dHToErrors1\Twist; */
	XXMatrixSub (&M[25], &M[26], &M[21]);

	/* Damping\output = diag (Damping\Damping) * PlusMinus2\output; */
	XXMatrixDiag (&M[52], &M[20]);
	XXMatrixMul (&M[19], &M[52], &M[25]);

	/* PlusMinus5\output = CartesianSpaceStiffness\effort - Damping\output; */
	XXMatrixSub (&M[27], &M[0], &M[19]);

	/* W_tooltip_00 = PlusMinus5\output; */
	XXMatrixMov (&M[34], &M[27]);

	}

	/* This function calculates the output equations of the model.
	 * These equations are not needed for calculation of the rates
	 * and are kept separate to make the dynamic set of equations smaller.
	 * These dynamic equations are called often more than one time for each
	 * integration step that is taken. This makes model computation much faster.
	 */
	inline void CartesianControllerModel::CalculateOutput (void)
	{
			/* CartesianSpaceStiffness\Htip0 = Htip0; */
	XXMatrixMov (&M[1], &M[31]);

	/* CartesianSpaceStiffness\HtipCC = HtipCC; */
	XXMatrixMov (&M[3], &M[33]);

	/* CartesianSpaceStiffness\dummy1 = CartesianSpaceStiffness\Kcc[1:3,1:3]; */
	M[12].mat[0] = M[7].mat[0];
	M[12].mat[1] = M[7].mat[1];
	M[12].mat[2] = M[7].mat[2];
	M[12].mat[3] = M[7].mat[6];
	M[12].mat[4] = M[7].mat[7];
	M[12].mat[5] = M[7].mat[8];
	M[12].mat[6] = M[7].mat[12];
	M[12].mat[7] = M[7].mat[13];
	M[12].mat[8] = M[7].mat[14];

	/* CartesianSpaceStiffness\dummy2 = CartesianSpaceStiffness\Kcc[1:3,4:6]; */
	M[13].mat[0] = M[7].mat[3];
	M[13].mat[1] = M[7].mat[4];
	M[13].mat[2] = M[7].mat[5];
	M[13].mat[3] = M[7].mat[9];
	M[13].mat[4] = M[7].mat[10];
	M[13].mat[5] = M[7].mat[11];
	M[13].mat[6] = M[7].mat[15];
	M[13].mat[7] = M[7].mat[16];
	M[13].mat[8] = M[7].mat[17];

	/* CartesianSpaceStiffness\dummy3 = CartesianSpaceStiffness\Kcc[4:6,4:6]; */
	M[14].mat[0] = M[7].mat[21];
	M[14].mat[1] = M[7].mat[22];
	M[14].mat[2] = M[7].mat[23];
	M[14].mat[3] = M[7].mat[27];
	M[14].mat[4] = M[7].mat[28];
	M[14].mat[5] = M[7].mat[29];
	M[14].mat[6] = M[7].mat[33];
	M[14].mat[7] = M[7].mat[34];
	M[14].mat[8] = M[7].mat[35];

	/* CartesianSpaceStiffness\costiffness1 = (0.5 * trace (CartesianSpaceStiffness\dummy1)) * eye (3) - CartesianSpaceStiffness\dummy1; */
	XXMatrixEye (&M[54]);
	XXScalarMatrixMul (&M[53], 0.5 * XXMatrixTrace (&M[12]), &M[54]);
	XXMatrixSub (&M[9], &M[53], &M[12]);

	/* CartesianSpaceStiffness\costiffness2 = (0.5 * trace (CartesianSpaceStiffness\dummy2)) * eye (3) - CartesianSpaceStiffness\dummy2; */
	XXMatrixEye (&M[56]);
	XXScalarMatrixMul (&M[55], 0.5 * XXMatrixTrace (&M[13]), &M[56]);
	XXMatrixSub (&M[10], &M[55], &M[13]);

	/* CartesianSpaceStiffness\costiffness3 = (0.5 * trace (CartesianSpaceStiffness\dummy3)) * eye (3) - CartesianSpaceStiffness\dummy3; */
	XXMatrixEye (&M[58]);
	XXScalarMatrixMul (&M[57], 0.5 * XXMatrixTrace (&M[14]), &M[58]);
	XXMatrixSub (&M[11], &M[57], &M[14]);

	/* CartesianSpaceStiffness\H1_0 = ((inverseH (Delay\input * CartesianSpaceStiffness\HtipCC) * CartesianSpaceStiffness\Htip0) * CartesianSpaceStiffness\HtipCC); */
	XXMatrixMul (&M[62], &M[35], &M[3]);
	XXMatrixInverseH (&M[61], &M[62]);
	XXMatrixMul (&M[60], &M[61], &M[1]);
	XXMatrixMul (&M[59], &M[60], &M[3]);
	XXMatrixMul (&M[8], &M[60], &M[3]);

	/* CartesianSpaceStiffness\skew21 = skew (CartesianSpaceStiffness\H1_0[1:3,4]); */
	M[63].mat[0] = M[8].mat[3];
	M[63].mat[1] = M[8].mat[7];
	M[63].mat[2] = M[8].mat[11];
	XXMatrixSkew (&M[17], &M[63]);

	/* CartesianSpaceStiffness\orientation21 = CartesianSpaceStiffness\H1_0[1:3,1:3]; */
	M[18].mat[0] = M[8].mat[0];
	M[18].mat[1] = M[8].mat[1];
	M[18].mat[2] = M[8].mat[2];
	M[18].mat[3] = M[8].mat[4];
	M[18].mat[4] = M[8].mat[5];
	M[18].mat[5] = M[8].mat[6];
	M[18].mat[6] = M[8].mat[8];
	M[18].mat[7] = M[8].mat[9];
	M[18].mat[8] = M[8].mat[10];

	/* CartesianSpaceStiffness\dummy4 = (2 * antisym (CartesianSpaceStiffness\costiffness1 * CartesianSpaceStiffness\orientation21) + antisym ((((CartesianSpaceStiffness\costiffness3 * transpose (CartesianSpaceStiffness\orientation21)) * CartesianSpaceStiffness\skew21) * CartesianSpaceStiffness\skew21) * CartesianSpaceStiffness\orientation21)) + 2 * antisym ((CartesianSpaceStiffness\costiffness2 * CartesianSpaceStiffness\skew21) * CartesianSpaceStiffness\orientation21); */
	XXMatrixMul (&M[67], &M[9], &M[18]);
	XXMatrixAsym (&M[66], &M[67], workarray);
	XXScalarMatrixMul (&M[65], 2.0, &M[66]);
	XXMatrixTranspose (&M[73], &M[18]);
	XXMatrixMul (&M[72], &M[11], &M[73]);
	XXMatrixMul (&M[71], &M[72], &M[17]);
	XXMatrixMul (&M[70], &M[71], &M[17]);
	XXMatrixMul (&M[69], &M[70], &M[18]);
	XXMatrixAsym (&M[68], &M[69], workarray);
	XXMatrixAdd (&M[64], &M[65], &M[68]);
	XXMatrixMul (&M[77], &M[10], &M[17]);
	XXMatrixMul (&M[76], &M[77], &M[18]);
	XXMatrixAsym (&M[75], &M[76], workarray);
	XXScalarMatrixMul (&M[74], 2.0, &M[75]);
	XXMatrixAdd (&M[15], &M[64], &M[74]);

	/* CartesianSpaceStiffness\dummy5 = ((transpose (CartesianSpaceStiffness\orientation21) * antisym (CartesianSpaceStiffness\costiffness3 * CartesianSpaceStiffness\skew21)) * CartesianSpaceStiffness\orientation21 + antisym (((CartesianSpaceStiffness\costiffness3 * transpose (CartesianSpaceStiffness\orientation21)) * CartesianSpaceStiffness\skew21) * CartesianSpaceStiffness\orientation21)) + 2 * antisym (CartesianSpaceStiffness\costiffness2 * CartesianSpaceStiffness\orientation21); */
	XXMatrixTranspose (&M[81], &M[18]);
	XXMatrixMul (&M[83], &M[11], &M[17]);
	XXMatrixAsym (&M[82], &M[83], workarray);
	XXMatrixMul (&M[80], &M[81], &M[82]);
	XXMatrixMul (&M[79], &M[80], &M[18]);
	XXMatrixTranspose (&M[88], &M[18]);
	XXMatrixMul (&M[87], &M[11], &M[88]);
	XXMatrixMul (&M[86], &M[87], &M[17]);
	XXMatrixMul (&M[85], &M[86], &M[18]);
	XXMatrixAsym (&M[84], &M[85], workarray);
	XXMatrixAdd (&M[78], &M[79], &M[84]);
	XXMatrixMul (&M[91], &M[10], &M[18]);
	XXMatrixAsym (&M[90], &M[91], workarray);
	XXScalarMatrixMul (&M[89], 2.0, &M[90]);
	XXMatrixAdd (&M[16], &M[78], &M[89]);

	/* CartesianSpaceStiffness\effort = -transpose (Adjoint (inverseH (CartesianSpaceStiffness\Htip0 * CartesianSpaceStiffness\HtipCC))) * [CartesianSpaceStiffness\dummy4[3,2]; CartesianSpaceStiffness\dummy4[1,3]; CartesianSpaceStiffness\dummy4[2,1]; CartesianSpaceStiffness\dummy5[3,2]; CartesianSpaceStiffness\dummy5[1,3]; CartesianSpaceStiffness\dummy5[2,1]]; */
	XXMatrixMul (&M[96], &M[1], &M[3]);
	XXMatrixInverseH (&M[95], &M[96]);
	XXMatrixAdjoint (&M[94], &M[95]);
	XXMatrixTranspose (&M[93], &M[94]);
	XXMatrixInv (&M[92], &M[93]);
	M[97].mat[0] = M[15].mat[7];
	M[97].mat[1] = M[15].mat[2];
	M[97].mat[2] = M[15].mat[3];
	M[97].mat[3] = M[16].mat[7];
	M[97].mat[4] = M[16].mat[2];
	M[97].mat[5] = M[16].mat[3];
	XXMatrixMul (&M[0], &M[92], &M[97]);

			}

	/* This function calculates the final equations of the model.
	 * These equations are calculated after all the calculations
	 * are performed
	 */
	inline void CartesianControllerModel::CalculateFinal (void)
	{
		
	}

	bool CartesianControllerModel::setPeriod(double seconds)
	{
			step_size = seconds;
			return true;
	}

	double CartesianControllerModel::getPeriod()
  {
      return step_size;
  }

}

