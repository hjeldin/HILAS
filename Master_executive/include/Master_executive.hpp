#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <string>
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

namespace YouBot
{
	typedef std_msgs::Float64MultiArray flat_matrix_t;

	class Master_executive: public RTT::TaskContext
	{
	public:
		Master_executive(const std::string& name);
		virtual ~Master_executive(void);

		void submitEnergyQuanta(double joules);
		double getEnergyState1();

		void setCartesianStiffness(std::vector<double> stiffness_c);

//		bool configureHook ();
		bool startHook ();
		void updateHook ();
		void stopHook ();

	protected:
		void setCartSpaceStiffness();

		RTT::OutputPort<flat_matrix_t> EnergyQuanta;
		RTT::InputPort<flat_matrix_t > EnergyState1;

		RTT::InputPort<flat_matrix_t > stiffness_slider;
		RTT::OutputPort<flat_matrix_t> CartSpaceStiffness;

	private:
		flat_matrix_t m_EnergyQuanta;
		flat_matrix_t m_EnergyState1;

		flat_matrix_t m_stiffness_slider;
		flat_matrix_t m_CartSpaceStiffness;
		flat_matrix_t m_CartSpaceStiffness_orig;

	};

}

