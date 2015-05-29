#ifndef __CONTROLIT_CORE_TESTS_CONTROLIT_TASK_TEST_HPP__
#define __CONTROLIT_CORE_TESTS_CONTROLIT_TASK_TEST_HPP__

#include <controlit/Task.hpp>

namespace controlit {
namespace example {

class SimpleTask : public controlit::Task
{
public:
    SimpleTask()
    {
        declareParameter("intVal", &intVal);
        declareParameter("realVal", &realVal);
        declareParameter("stringVal", &stringVal);
        declareParameter("matrixVal", &matrixVal);
        declareParameter("vectorVal", &vectorVal);
    }
  
    virtual bool init(ControlModel & model)
    {
        return true;
    }
  
    virtual bool reinit(ControlModel & model)
    {
        return init(model);
    }
  
    virtual bool updateStateImpl(ControlModel * model, TaskState * taskState)
    {
        return true;
    }
  
    virtual bool getJtask(ControlModel & model, Matrix & Jt)
    {
        Jt.setIdentity();
        return true;
    }
  
    virtual bool getCommand(ControlModel & model, TaskCommand & command)
    {
        return true;
    }
  
protected:
    int intVal;
    double realVal;
    std::string stringVal;
    Matrix matrixVal;
    Vector vectorVal;
};

} // namespace example
} // namespace controlit

#endif // __CONTROLIT_CORE_TESTS_CONTROLIT_TASK_TEST_HPP__