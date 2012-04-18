#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <string>

#include <youbot/YouBotBase.hpp>
#include <youbot/YouBotManipulator.hpp>
#include "YouBotJointStatusRecorder.h"

namespace YouBot
{
	using namespace RTT;
	using namespace youbot;

	class YouBot_configurator: public TaskContext
	{
	public:
		YouBot_configurator(const string& name);
		virtual ~YouBot_configurator();
		virtual bool setDefinedConfiguration();
		virtual double pidTunning(double signalLevel, int slaveNumber);
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		double tolerance;
		int initPID[4];
		int maxPID[4];
		int PID[4];
		int oldPID[4];
		int stepPID[4];
		string dataFile;
		double w1;
		double w2;
		double w3;
		quantity<si::current> step;
		int step_counter;
		int int_err;
		int int_err_prev;
		double tu;
		double t;
		int param_index;
	private:
		enum states{
			PARAMETER_CHANGE,
			TEST_SIGNAL_LOW,
			TEST_SIGNAL_HIGH,
			NOT_RUNNABLE
		};
		YouBot_configurator::states current_state;
		YouBotManipulator* m_manipulator;
		YouBotBase* m_base;
		YouBotJointStatusRecorder* statusRecoder;
		YouBotJoint* joint;
		void changePIDParameters(int P,int I, int D);
		double runTest(int P,int I, int D);
		double stepResponseWithError();


	};
}

