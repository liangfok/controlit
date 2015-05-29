#include <gtest/gtest.h>

#include <controlit/CompoundTaskFactory.hpp>
#include <ros/package.h> // for ros::package::getPath(...)

class CompoundTaskFactoryTest : public ::testing::Test
{
protected:
    virtual void SetUp() {}
    virtual void TearDown() {}
};

/* ----------------------------------------------------------------------------
 *  TaskFactory tests
 * --------------------------------------------------------------------------*/
TEST_F(CompoundTaskFactoryTest, Load)
{
    std::string source_dir = ros::package::getPath("controlit");

    controlit::CompoundTaskFactory compoundTaskFactory;
    controlit::CompoundTask* ct = compoundTaskFactory.loadFromFile(source_dir + "/tests/factories/compound_task_factory_test.yaml");

    EXPECT_TRUE(ct != NULL) << "ct was NULL!";
    controlit::Parameter* intParam = ct->lookupParameter("t1.intVal");
    EXPECT_TRUE(intParam != NULL) << "intParam is NULL";
    EXPECT_TRUE(*(intParam->getInteger()) == 3) << "Value: " << *(intParam->getInteger());

    controlit::Parameter* realParam = ct->lookupParameter("t1.realVal");
    EXPECT_TRUE(realParam != NULL);
    EXPECT_TRUE(*(realParam->getReal()) == -2.5) << "Value: " << *(realParam->getReal());

    controlit::Parameter* stringParam = ct->lookupParameter("t1.stringVal");
    EXPECT_TRUE(stringParam != NULL);
    EXPECT_TRUE(*(stringParam->getString()) == "world") << *(realParam->getString());

    intParam = ct->lookupParameter("t2.intVal");
    EXPECT_TRUE(intParam != NULL);
    ASSERT_EQ(*(intParam->getInteger()), -3);

    realParam = ct->lookupParameter("t2.realVal");
    EXPECT_TRUE(realParam != NULL);
    ASSERT_EQ(*(realParam->getReal()), 4.5);

    stringParam = ct->lookupParameter("t2.stringVal");
    EXPECT_TRUE(stringParam != NULL);
    ASSERT_STREQ(stringParam->getString()->c_str(), "world2");
}
