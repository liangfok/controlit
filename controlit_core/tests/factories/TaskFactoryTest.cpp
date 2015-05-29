#include <gtest/gtest.h>

#include <controlit/TaskFactory.hpp>
#include <controlit/Parameter.hpp>
#include <ros/package.h> // for ros::package::getPath(...)

class TaskFactoryTest : public ::testing::Test
{
protected:
	virtual void SetUp() {}
	virtual void TearDown() {}
};

/* ----------------------------------------------------------------------------
 *	TaskFactory tests
 * --------------------------------------------------------------------------*/
TEST_F(TaskFactoryTest, Load)
{
  std::string source_dir = ros::package::getPath("controlit");

  controlit::TaskFactory taskFactory;
  controlit::Task* task = taskFactory.loadFromFile(source_dir + "/tests/factories/task_factory_test.yaml");

  EXPECT_TRUE(task != NULL);

  controlit::Parameter* intParam = task->lookupParameter("intVal");
  EXPECT_TRUE(intParam != NULL);
  EXPECT_TRUE(*(intParam->getInteger()) == 10);

  controlit::Parameter* realParam = task->lookupParameter("realVal");
  EXPECT_TRUE(realParam != NULL);
  EXPECT_TRUE(*(realParam->getReal()) == -5.4);

	controlit::Parameter* stringParam = task->lookupParameter("stringVal");
	EXPECT_TRUE(stringParam != NULL);
	EXPECT_TRUE(*(stringParam->getString()) == "hello");

	controlit::Parameter* vectorParam = task->lookupParameter("vectorVal");
	EXPECT_TRUE(vectorParam != NULL);
	EXPECT_TRUE((*(vectorParam->getVector())).norm()==1);

	controlit::Parameter* matrixParam = task->lookupParameter("matrixVal");
	EXPECT_TRUE(matrixParam != NULL);
	EXPECT_TRUE((*(matrixParam->getMatrix())).rows()==2);
	EXPECT_TRUE((*(matrixParam->getMatrix())).cols()==4);
	EXPECT_TRUE((*(matrixParam->getMatrix()))(0,0)==1.0);
}
