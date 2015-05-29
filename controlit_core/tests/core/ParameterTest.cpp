#include <gtest/gtest.h>

#include <controlit/ParameterListener.hpp>
#include <controlit/Parameter.hpp>

class ParameterTest : public ::testing::Test
{
protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

namespace param_test {

/* ----------------------------------------------------------------------------
 *  Parameter tests
 * --------------------------------------------------------------------------*/
template<class T>
controlit::Parameter* createParameter(const std::string& name, T* value)
{
  return NULL;
}

template<>
controlit::Parameter* createParameter<int>(const std::string& name, int* value)
{
  return new controlit::IntegerParameter(name, controlit::Parameter::Flag::Default, value);
}

template<>
controlit::Parameter* createParameter<double>(const std::string& name, double* value)
{
  return new controlit::RealParameter(name, controlit::Parameter::Flag::Default, value);
}

template<>
controlit::Parameter* createParameter<std::string>(const std::string& name, std::string* value)
{
  return new controlit::StringParameter(name, controlit::Parameter::Flag::Default, value);
}

template<>
controlit::Parameter* createParameter<Vector>(const std::string& name, Vector* value)
{
  return new VectorParameter(name, controlit::Parameter::Flag::Default, value);
}

template<>
controlit::Parameter* createParameter<Matrix>(const std::string& name, Matrix* value)
{
  return new MatrixParameter(name, controlit::Parameter::Flag::Default, value);
}

template<>
controlit::Parameter* createParameter< std::vector<std::string> >(const std::string& name, std::vector<std::string> * value)
{
  return new controlit::ListParameter(name, controlit::Parameter::Flag::Default, value);
}

template<>
controlit::Parameter* createParameter<controlit::BindingConfig>(const std::string& name, controlit::BindingConfig* value)
{
  return new controlit::BindingConfigParameter(name, controlit::Parameter::Flag::Default, value);
}

template<class T>
bool testSetParameter(T val)
{
  // Create a parameter
  T local;
  controlit::Parameter* p = createParameter<T>("param", &local);
  if (p == NULL) return false;

  // Test whether we can set it to the passed in val
  p->set(val);
  bool result = (local == val);
  delete p;
  return result;
}

} // end param_test namespace

TEST_F(ParameterTest, SetValue)
{
  EXPECT_TRUE(param_test::testSetParameter<int>(-99));
  EXPECT_TRUE(param_test::testSetParameter<double>(3.1));
  EXPECT_TRUE(param_test::testSetParameter<std::string>("hello"));
  EXPECT_TRUE(param_test::testSetParameter<Matrix>(Matrix::Zero(2,2)));
  EXPECT_TRUE(param_test::testSetParameter<Vector>(Vector::Zero(3)));
  EXPECT_TRUE(param_test::testSetParameter<std::vector<std::string> >(std::vector<std::string>(3,"test")));
  controlit::BindingConfig binding = controlit::BindingConfig("ROS", "/joint_states", "geometry_msgs::JointState", controlit::BindingConfig::Input);
  binding.addParameter("a_param");
  EXPECT_TRUE(param_test::testSetParameter<controlit::BindingConfig>(binding));
}

/* ----------------------------------------------------------------------------
 *  Listener functionality tests
 * --------------------------------------------------------------------------*/
class MySimpleIntParamListener
{
public:
  MySimpleIntParamListener() : val(0) {}

  virtual void update(controlit::Parameter const& param)
  {
    if (param.isType(controlit::PARAMETER_TYPE_INTEGER))
    {
      val = *(param.getInteger());
    }
  }

  int val;
};

TEST_F(ParameterTest, ParameterListener)
{
  int aInt = 3;
  controlit::Parameter* p = param_test::createParameter<int>("aInt", &aInt);

  // Create some listeners
  MySimpleIntParamListener l;
  p->addListener(boost::bind(&MySimpleIntParamListener::update, &l, _1));

  // update value and check that listener got it
  p->set(10);
  EXPECT_TRUE(aInt == 10);
  EXPECT_TRUE(l.val == 10);

  delete p;
}
