/**********************************************************
 * This file is generated by 20-sim ANSI-C Code Generator
 *
 *  file:  src\common\xxfuncs.cpp
 *  subm:  YouBot_control
 *  model: RCCMotionStack
 *  expmt: RCCMotionStack
 *  date:  November 17, 2011
 *  time:  1:52:23 pm
 *  user:  Campuslicentie
 *  from:  Universiteit Twente
 *  build: 4.1.2.2
 **********************************************************/

/* This file describes the additional SIDOPS functions
   that may appear in the generated equation model (.e file).

   For flexibility, ANSI-C is created, and typedefs are used
   for integers and doubles, see the xxfuncs.h file for more
   information on these types.

   This means that all used functions follow the ANSI definition
   (see the help file of Visual C++ 5 for more info).

   Please check the math.h file of your particular compiler
   to see if this is indeed the case. Otherwise, you might have
   to adapt the used functions below to obtain the same behavior.

*/

#ifndef XX_FUNCS_H

/* the system include files */
#include <stdlib.h>
#include <math.h>

/* our own include files */
#include "xxfuncs.h"

/* constants that are used in our functions below */
XXDouble xx_logarithm_2 =  0.6931471805599;
XXDouble xx_logarithm_10 = 2.3025850929940;

/* the 20-sim functions */
XXDouble XXAbsolute (XXDouble argument)
{
	return fabs (argument);
}

XXDouble XXArcCosineHyperbolic (XXDouble argument)
{
	return log (argument + sqrt(argument * argument - 1.0));
}

XXDouble XXArcSineHyperbolic (XXDouble argument)
{
 	return log (argument + sqrt(argument * argument + 1.0));
}

XXDouble XXArcTangentHyperbolic (XXDouble argument)
{
	return log (sqrt( argument * argument + 1.0) - sqrt(argument * argument - 1.0));
}

XXDouble XXExponent2 (XXDouble argument)
{
	return exp (argument * xx_logarithm_2);
}

XXDouble XXExponent10 (XXDouble argument)
{
	return exp (argument * xx_logarithm_10);
}

XXDouble XXIntegerDivide (XXDouble argument1, XXDouble argument2)
{
	XXInteger value;

	value = (XXInteger) (argument1 / argument2);
	return (XXDouble) value;
}

XXDouble XXIntegerModulo (XXDouble argument1, XXDouble argument2)
{
	XXDouble value;

	value = (XXInteger)(argument1 / argument2);
	return argument1 - (value  * argument2);
}

XXDouble XXLogarithm2 (XXDouble argument)
{
	return log (argument) / xx_logarithm_2;
}

XXDouble XXLogarithm10 (XXDouble argument)
{
	return log (argument) / xx_logarithm_10;
}

XXDouble XXPow2 (XXDouble argument)
{
	return argument * argument;
}

XXDouble XXPower (XXDouble argument1, XXDouble argument2)
{
	return pow (argument1, argument2);
}

XXDouble XXRandom (XXDouble argument)
{
	XXDouble value;

	value = (XXDouble) rand() / (XXDouble) RAND_MAX - 0.5;
	return argument * 2.0 * value;
}

XXDouble XXRamp (XXDouble argument, XXDouble time)
{
	XXDouble value;

	if (time < argument)
		value = 0.0;
	else
		value = time - argument;
	return value;
}

XXDouble XXSign (XXDouble argument)
{
	XXDouble value;
	if (argument < 0.0)
		value = -1.0;
	else
		if (argument == 0.0)
			value = 0.0;
		else
			value = 1.0;
	return value;
}

XXDouble XXStep (XXDouble argument, XXDouble time)
{
	XXDouble value;

	if (time < argument)
		value = 0.0;
	else
		value = 1.0;
	return value;
}

XXDouble XXImpulse (XXDouble arg1, XXDouble arg2, XXDouble time, XXDouble stepsize)
{
	XXDouble value;

	if (stepsize <= 0.0 || arg2 <= 0.0)
		value = 0.0;
	else
	{
		if ((time < arg1) || (time > (arg1 + stepsize)))
			value = 0.0;
		else
		{
			if (stepsize < arg2)
				value = (1.0 / arg2) * (XXStep (arg1, time) - XXStep (arg1 + arg2, time));
			else
				value = (1.0 / stepsize) * (XXStep (arg1, time) - XXStep (arg1 + stepsize, time));
		}
	}
	return value;
}

XXDouble XXXor(XXDouble argument1, XXDouble argument2)
{
	return (argument1 || argument2) && !(argument1 && argument2);
}


XXDouble XXRound (XXDouble argument)
{
	XXDouble leftOver, result;

	leftOver = argument - (XXInteger) argument;
	if (fabs (leftOver) < 0.5)
	{
		result = (XXDouble) ((XXInteger) argument);
	}
	else
	{
		if (argument >= 0)
			result = ceil (argument);
		else
		{
			result = floor (argument);
		}
	}
	return result;
}


/* 20-sim stubs. Implement them yourself if needed */
XXDouble XXData (XXString name, XXInteger column, XXInteger id)
{
	return 0;
}

XXDouble XXTable (XXString name, XXDouble argument, XXInteger id)
{
	return 0;
}

XXBoolean XXEvent (XXDouble argument, XXInteger id)
{
	return 0;
}

XXBoolean XXEventUp (XXDouble argument, XXInteger id)
{
	return 0;
}

XXBoolean XXEventDown (XXDouble argument, XXInteger id)
{
	return 0;
}

XXBoolean XXFrequencyEvent (XXDouble argument, XXInteger id)
{
	return 0;
}

XXBoolean XXFrequencyEvent1 (XXDouble argument1, XXDouble argument2, XXInteger id)
{
	return 0;
}

XXBoolean XXTimeEvent (XXDouble argument, XXInteger id)
{
	return 0;
}

XXDouble XXTimeDelay (XXDouble argument, XXDouble time, XXInteger id)
{
	return 0;
}

XXBoolean XXWarning (XXString message, XXInteger id)
{
	return 0;
}

XXBoolean XXStopSimulation (XXString message, XXInteger id)
{
	return 0;
}

#endif

