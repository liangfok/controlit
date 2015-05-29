#ifndef _CONTROLIT_CORE_TESTS_CONTROLIT_CONSTRAINT_TEST_HPP_
#define _CONTROLIT_CORE_TESTS_CONTROLIT_CONSTRAINT_TEST_HPP_

#include <controlit/Constraint.hpp>

namespace controlit {
namespace example {

class SimpleConstraint : public controlit::Constraint
{
public:
	SimpleConstraint() : controlit::Constraint()
	{
		setupParameters();
	}

	virtual void init(RigidBodyDynamics::Model& robot)
	{
		controlit::Constraint::init(robot);
	}

	virtual void getJacobian(RigidBodyDynamics::Model& robot, const Vector& Q, Matrix& Jc)
	{
		return;
	}

protected:

	virtual void setupParameters()
	{
		declareParameter("intVal", &intVal);
		declareParameter("realVal", &realVal);
		declareParameter("stringVal", &stringVal);

		Constraint::setupParameters();
	}

	int intVal;
	double realVal;
	std::string stringVal;
};

} // namespace example
} // namespace controlit

#endif