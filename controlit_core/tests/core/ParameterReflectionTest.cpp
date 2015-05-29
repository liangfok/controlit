#include <gtest/gtest.h>

#include <controlit/ParameterReflection.hpp>

class ParameterReflectionTest : public ::testing::Test
{
protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

/* ----------------------------------------------------------------------------
 *  Parameter testing
 * --------------------------------------------------------------------------*/
template<class T>
void testGetParamByName(controlit::ParameterType type)
{
  controlit::ParameterReflection pr("pr","");

  EXPECT_TRUE(pr.addParameter("param", new T) != NULL);
  controlit::Parameter* p = pr.lookupParameter("param");
  EXPECT_TRUE(p != NULL);
  EXPECT_TRUE(p->name() == "param");
  EXPECT_TRUE(p->type() == type);
}

TEST_F(ParameterReflectionTest, GetParameterByName)
{
  testGetParamByName<int>(controlit::PARAMETER_TYPE_INTEGER);
  testGetParamByName<std::string>(controlit::PARAMETER_TYPE_STRING);
  testGetParamByName<double>(controlit::PARAMETER_TYPE_REAL);
  testGetParamByName<Vector>(controlit::PARAMETER_TYPE_VECTOR);
  testGetParamByName<Matrix>(controlit::PARAMETER_TYPE_MATRIX);
  testGetParamByName<std::vector<std::string> >(controlit::PARAMETER_TYPE_LIST);
}

template<class T>
void testGetParamByNameAndType(controlit::ParameterType type)
{
  controlit::ParameterReflection pr("pr","");

  EXPECT_TRUE(pr.addParameter("param", new T) != NULL);
  controlit::Parameter* p = pr.lookupParameter("param", type);
  EXPECT_TRUE(p != NULL);
  EXPECT_TRUE(p->name() == "param");
  EXPECT_TRUE(p->type() == type);
}

TEST_F(ParameterReflectionTest, GetParameterByNameAndType)
{
  testGetParamByNameAndType<int>(controlit::PARAMETER_TYPE_INTEGER);
  testGetParamByNameAndType<std::string>(controlit::PARAMETER_TYPE_STRING);
  testGetParamByNameAndType<double>(controlit::PARAMETER_TYPE_REAL);
  testGetParamByNameAndType<Vector>(controlit::PARAMETER_TYPE_VECTOR);
  testGetParamByNameAndType<Matrix>(controlit::PARAMETER_TYPE_MATRIX);
  testGetParamByNameAndType<std::vector<std::string> >(controlit::PARAMETER_TYPE_LIST);
}

/* ----------------------------------------------------------------------------
 *  Event testing
 * --------------------------------------------------------------------------*/
TEST_F(ParameterReflectionTest, AddEvents)
{
  controlit::ParameterReflection pr("pr","");

  // Add an always true event
  EXPECT_TRUE(pr.addEvent("e1", "1"));
  EXPECT_TRUE(pr.lookupParameter("e1") != NULL);

  // Add an always false event
  EXPECT_TRUE(pr.addEvent("e2", "0"));
  EXPECT_TRUE(pr.lookupParameter("e2") != NULL);

  // Add an event which relies on previously defined parameters
  EXPECT_TRUE(pr.addEvent("e3", "e1 and e2"));
  EXPECT_TRUE(pr.lookupParameter("e3") != NULL);

  // This event declaration should fail because e3 is already defined
  EXPECT_FALSE(pr.addEvent("e3", "e4 or e2"));
}

namespace event_test {

class MyParameterReflection : public controlit::ParameterReflection
{
public:
  MyParameterReflection() : controlit::ParameterReflection("type","pr") {}
  bool update() {return emitEvents();}
};

class EventTester
{
public:
  virtual void update(std::string const& name)
  {
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

} // namespace event_test

TEST_F(ParameterReflectionTest, SimpleLogic)
{
  event_test::MyParameterReflection pr;
  event_test::EventTester l;
  pr.addListener(boost::bind(&event_test::EventTester::update, &l, _1));

  // Define the events 'true' and 'false'.. 'false' should never be emitted
  pr.addEvent("true", "1");
  l.assertTrue("pr.true");
  pr.addEvent("false", "0");
  l.assertFalse("pr.false");

  // Reference existing parameter.. again 'e2' should never be emitted
  pr.addEvent("e1", "true");
  l.assertTrue("pr.e1");
  pr.addEvent("e2", "false");
  l.assertFalse("pr.e2");

  // More complex, nested logic
  pr.addEvent("e3", "e1 and e1"); // T && T = T
  l.assertTrue("pr.e3");
  pr.addEvent("e4", "e1 and e2"); // T && F = F
  l.assertFalse("pr.e4");
  pr.addEvent("e5", "e2 and e2"); // F && F = F
  l.assertFalse("pr.e5");

  // More complex, nested logic
  pr.addEvent("e6", "e1 or e1");  // T || T = T
  l.assertTrue("pr.e6");
  pr.addEvent("e7", "e1 or e2");  // T || F = T
  l.assertTrue("pr.e7");
  pr.addEvent("e8", "e2 or e2");  // F || F = F
  l.assertFalse("pr.e8");

  // Really nested logic
  pr.addEvent("e9", "(e7 or e8) and (e6 and e3)");  // (T || F) && (T && T) = T && T = T
  l.assertTrue("pr.e9");
  pr.addEvent("e10", "(e9 and e8) or (e5 and e7)"); // (T && F) || (F && T) = F || F = F
  l.assertFalse("pr.e10");

  if (!pr.update()) ADD_FAILURE() << "Problems updating parameter reflection object";
}

TEST_F(ParameterReflectionTest, ParameterLogic)
{
  event_test::MyParameterReflection pr;
  event_test::EventTester l;
  pr.addListener(boost::bind(&event_test::EventTester::update, &l, _1));

  // Add some fake parameters
  pr.addParameter("error_bound", new double(0.1));
  pr.addParameter("error_dot_bound", new double(0.05));
  pr.addParameter("t1_error", new double(-0.09));
  pr.addParameter("t1_error_dot", new double(0.03));
  pr.addParameter("t2_error", new double(0.05));
  pr.addParameter("t2_error_dot", new double(-0.11));

  // Define task events
  pr.addEvent("t1_converged", "abs(t1_error) < error_bound");
  l.assertTrue("pr.t1_converged");
  pr.addEvent("t1_quiescent", "abs(t1_error_dot) < error_dot_bound");
  l.assertTrue("pr.t1_quiescent");
  pr.addEvent("t2_converged", "abs(t2_error) < error_bound");
  l.assertTrue("pr.t2_converged");
  pr.addEvent("t2_quiescent", "abs(t2_error_dot) < error_dot_bound");
  l.assertFalse("pr.t2_quiescent");

  // Define some 'skill' events
  pr.addEvent("converged", "t1_converged and t2_converged");
  l.assertTrue("pr.converged");
  pr.addEvent("quiescent", "t1_quiescent and t2_quiescent");
  l.assertFalse("pr.quiescent");
  pr.addEvent("t1_complete", "t1_converged and t1_quiescent");
  l.assertTrue("pr.t1_complete");
  pr.addEvent("t2_complete", "t2_converged and t2_quiescent");
  l.assertFalse("pr.t2_complete");
  pr.addEvent("complete", "t1_complete or t2_complete");
  l.assertTrue("pr.complete");

  if (!pr.update()) ADD_FAILURE() << "Problems updating parameter reflection object";
}

#include <controlit/parser/yaml_parser.hpp>

class YamlEncodeTest : public controlit::ParameterReflection
{
public:
  YamlEncodeTest() :
    controlit::ParameterReflection("type", "name"),
    aReal(0.0),
    aInt(0),
    aString(""),
    aVector(Vector::Zero(2)),
    aMatrix(Matrix::Zero(2,2)),
    aList(std::vector<std::string>(2,""))
  {
    declareParameter("aReal", &aReal);
    declareParameter("aInt", &aInt);
    declareParameter("aString", &aString);
    declareParameter("aVector", &aVector);
    declareParameter("aMatrix", &aMatrix);
    declareParameter("aList", &aList);
  };

  double aReal;
  int aInt;
  std::string aString;
  Vector aVector;
  Matrix aMatrix;
  std::vector<std::string> aList;
};

TEST_F(ParameterReflectionTest, YamlDecode)
{
  const std::string yaml_string =
      "type: opspace::SelectedJointPostureTask\n"
      "name: t1\n"
      "parameters:\n"
    "   - name: aInt\n"
    "     type: int\n"
    "     value: 3\n"
    "   - name: aMatrix\n"
    "     type: matrix\n"
    "     value: \n"
    "      - [1, 0]\n"
    "      - [0, 1]\n"
    "   - name: aReal\n"
    "     type: real\n"
    "     value: -2.3\n"
    "   - name: aString\n"
    "     type: string\n"
    "     value: hello\n"
    "   - name: aVector\n"
    "     type: vector\n"
    "     value: [1, 1]\n"
    "   - name: aList\n"
    "     type: list\n"
    "     value: ['one', 'two']\n"
    "events:\n"
    "   - name: converged\n"
    "     expression: abs(error) < 0.1\n"
    "   - name: e1\n"
    "     expression: e1 and e2 or e3\n"
    "bindings:\n"
    "   - middleware: ROS\n"
    "     topic: /some/ros/topic/name\n"
    "     direction: input\n"
    "     parameters: [aInt, aMatrix, aReal]\n";

  YAML::Node doc;
  drc::common::yaml_read_string(yaml_string, doc);

  // YamlEncodeTest extends controlit::ParameterReflection; it is defined above.
  std::shared_ptr<YamlEncodeTest> yamlEncodeTest(new YamlEncodeTest);
  std::shared_ptr<controlit::ParameterReflection> pr(yamlEncodeTest);
  parse_parameter_reflection(doc, pr.get(), 1);
  YamlEncodeTest* test = static_cast<YamlEncodeTest*>(pr.get());

  EXPECT_TRUE(test->aInt == 3);
  EXPECT_TRUE(test->aReal == -2.3);
  EXPECT_TRUE(test->aString == "hello");
  EXPECT_TRUE(test->aVector == Vector::Ones(2));
  EXPECT_TRUE(test->aMatrix == Matrix::Identity(2,2));
  EXPECT_TRUE(test->aList[0] == "one");
  controlit::ParameterReflection::ParameterList_t bindings = pr->lookupParameters(controlit::PARAMETER_TYPE_BINDING);
  EXPECT_TRUE(bindings.size() == 1);

  YAML::Emitter emitter;
  emit_parameter_reflection(emitter, pr.get(), 1);
  //std::cout << emitter << std::endl;
}
