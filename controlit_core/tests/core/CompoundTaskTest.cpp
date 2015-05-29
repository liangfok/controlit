#include <gtest/gtest.h>

#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/CompoundTask.hpp>
#include <controlit/Task.hpp>
#include <controlit/TaskState.hpp>
#include <controlit/ControlModel.hpp>

using Vector;
using controlit::Task;
using controlit::TaskState;
using controlit::ControlModel;
using controlit::CompoundTask;
using controlit::EventListener;
using Vector;

class CompoundTaskTest : public ::testing::Test
{
protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

/* ----------------------------------------------------------------------------
 *  CompoundTask functionality
 * --------------------------------------------------------------------------*/
namespace compound_task_test {

class MyTask : public controlit::Task
{
public:

  MyTask(std::string const& name,
         double err,
         double err_dot) :
    controlit::Task(name, controlit::ACCELERATION, new TaskState(), new TaskState()), error(err), error_dot(err_dot)
  {
    // Add some more parameters
    declareParameter("error", &error);
    declareParameter("error_dot", &error_dot);

    // Add some events
    addEvent("converged", "abs(error) < 0.1");
    addEvent("quiescent", "abs(error_dot) < 0.1");
  }

  virtual bool init(const ControlModel& model)
  {
    return true;
  }

  bool updateStateImpl(ControlModel * model, TaskState * taskState)
  {
    return true;
  }

  bool getCommand(ControlModel & model, TaskCommand & command)
  {
    return true;
  }

protected:
  double error;
  double error_dot;
};

class MyCompoundTask : public CompoundTask
{
public:
  MyCompoundTask(std::string const& name) : CompoundTask(name) {}
};

class EventTester : public EventListener
{
public:
  virtual void update(std::string const& name)
  {
    //std::cout << "got event: " << name << std::endl;
    EXPECT_TRUE(true_event_names.find(name) != true_event_names.end()) << "event name: " << name;
    EXPECT_TRUE(false_event_names.find(name) == false_event_names.end()) << "event name: " << name;
  }

  void assertTrue(const std::string& name)
  {
    true_event_names.insert(name);
  }
  void assertFalse(const std::string& name)
  {
    false_event_names.insert(name);
  }
  std::set<std::string> true_event_names;
  std::set<std::string> false_event_names;
};

}

#include <controlit/parser/yaml_parser.hpp>

TEST_F(CompoundTaskTest, PuttingItAllTogether)
{
  // t1 - converged, quiescent
  Task * t1(new compound_task_test::MyTask("t1",0.01, -0.03));
  // t2 - quiescent
  Task * t2(new compound_task_test::MyTask("t2",0.11, 0.05));
  // t3 - converged
  Task * t3(new compound_task_test::MyTask("t3",-0.05, 0.13));

  Task * t4(new compound_task_test::MyTask("t4",0.11, 0.05));
  Task * t5(new compound_task_test::MyTask("t5",-0.05, 0.13));

  Task * t6(new compound_task_test::MyTask("t6",0.11, 0.05));
  Task * t7(new compound_task_test::MyTask("t7",-0.05, 0.13));

  compound_task_test::MyCompoundTask ct("ct");
  ct.addTask(t1);
  ct.addTask(t2);
  ct.addTask(t3);
  ct.addTask(t4,0);
  ct.addTask(t5,3);
  ct.addTask(t6,3);
  ct.addTask(t7,3);

  compound_task_test::EventTester et;
  //EXPECT_TRUE(ct.addListener(&et));
  ct.addListener(boost::bind(&compound_task_test::EventTester::update, &et, _1));

  // Add some events to ct (the compound task)
  ct.addEvent("e1", "t1.converged and t3.converged");
  et.assertTrue("e1");
  ct.addEvent("e2", "t1.quiescent and t2.quiescent");
  et.assertTrue("e2");
  ct.addEvent("e3", "t1.converged and t1.quiescent");
  et.assertTrue("e3");

  ct.addEvent("e4", "t1.converged and t2.converged");
  et.assertFalse("e4");
  ct.addEvent("e5", "t2.quiescent and t3.quiescent");
  et.assertFalse("e5");
  ct.addEvent("e6", "(t1.converged and t1.quiescent) or (t3.converged)");
  et.assertTrue("e6");
  // .. I could go on..

  YAML::Emitter emitter;
  ct.saveConfig(emitter, 1);
  //std::cout << emitter << std::endl;
}
