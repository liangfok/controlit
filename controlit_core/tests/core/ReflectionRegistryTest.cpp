#include <gtest/gtest.h>

#include <controlit/ReflectionRegistry.hpp>

class ReflectionRegistryTest : public ::testing::Test
{
protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

/* ----------------------------------------------------------------------------
 *  Get nested parameters tests
 * --------------------------------------------------------------------------*/
TEST_F(ReflectionRegistryTest, AddRegistries)
{
  controlit::ReflectionRegistry rr("rr", "rr");

  // Add some registries
  EXPECT_TRUE(rr.addParameterCollection( std::shared_ptr<controlit::ParameterReflection>(new controlit::ParameterReflection("", "pr1"))) );
  EXPECT_TRUE(rr.addParameterCollection( std::shared_ptr<controlit::ParameterReflection>(new controlit::ParameterReflection("", "pr2"))) );
  EXPECT_TRUE(rr.addParameterCollection( std::shared_ptr<controlit::ParameterReflection>(new controlit::ParameterReflection("", "pr3"))) );

  // Can't add task with same name
  EXPECT_FALSE(rr.addParameterCollection( std::shared_ptr<controlit::ParameterReflection>(new controlit::ParameterReflection("", "pr3"))) );
}

TEST_F(ReflectionRegistryTest, GetParameterReflection)
{
  std::shared_ptr<controlit::ParameterReflection> pr( new controlit::ParameterReflection("pr", "pr") );
  controlit::ReflectionRegistry rr("rr", "rr");
  rr.addParameterCollection(pr);

  std::weak_ptr<controlit::ParameterReflection> ptr = rr.getParameterCollection("pr");

  EXPECT_TRUE(ptr.lock().get());

  ptr = rr.getParameterCollection("dne");
  EXPECT_TRUE(ptr.lock().get() == NULL);
}

namespace rr_tests
{

template<class T>
void testGetParamByName(controlit::ParameterType type)
{
  std::shared_ptr<controlit::ParameterReflection> pr( new controlit::ParameterReflection("pr", "pr") );
  std::shared_ptr<controlit::ReflectionRegistry> rr( new controlit::ReflectionRegistry("rr", "rr") );

  EXPECT_TRUE(pr->addParameter("param", new T) != NULL);
  EXPECT_TRUE(rr->addParameterCollection(pr));

  controlit::Parameter* p = rr->lookupParameter("pr.param");
  EXPECT_TRUE(p != NULL);
  EXPECT_TRUE(p->name() == "param");
  EXPECT_TRUE(p->type() == type);
}

}

TEST_F(ReflectionRegistryTest, GetParameterByName)
{
  rr_tests::testGetParamByName<int>(controlit::PARAMETER_TYPE_INTEGER);
  rr_tests::testGetParamByName<std::string>(controlit::PARAMETER_TYPE_STRING);
  rr_tests::testGetParamByName<double>(controlit::PARAMETER_TYPE_REAL);
  rr_tests::testGetParamByName<Vector>(controlit::PARAMETER_TYPE_VECTOR);
  rr_tests::testGetParamByName<Matrix>(controlit::PARAMETER_TYPE_MATRIX);
}

namespace rr_tests
{

template<class T>
void testGetParamByNameAndType(controlit::ParameterType type)
{
  std::shared_ptr<controlit::ParameterReflection> pr( new controlit::ParameterReflection("pr", "pr") );
  std::shared_ptr<controlit::ReflectionRegistry> rr( new controlit::ReflectionRegistry("rr", "rr") );

  EXPECT_TRUE(pr->addParameter("param", new T) != NULL);
  EXPECT_TRUE(rr->addParameterCollection(pr));

  controlit::Parameter* p = rr->lookupParameter("pr.param", type);
  EXPECT_TRUE(p != NULL);
  EXPECT_TRUE(p->name() == "param");
  EXPECT_TRUE(p->type() == type);
}

}

TEST_F(ReflectionRegistryTest, GetParameterByNameAndType)
{
  rr_tests::testGetParamByNameAndType<int>(controlit::PARAMETER_TYPE_INTEGER);
  rr_tests::testGetParamByNameAndType<std::string>(controlit::PARAMETER_TYPE_STRING);
  rr_tests::testGetParamByNameAndType<double>(controlit::PARAMETER_TYPE_REAL);
  rr_tests::testGetParamByNameAndType<Vector>(controlit::PARAMETER_TYPE_VECTOR);
  rr_tests::testGetParamByNameAndType<Matrix>(controlit::PARAMETER_TYPE_MATRIX);
}

TEST_F(ReflectionRegistryTest, UpdateEvent)
{
  std::shared_ptr<controlit::ParameterReflection> pr( new controlit::ParameterReflection("pr", "pr") );
  std::shared_ptr<controlit::ReflectionRegistry> rr( new controlit::ReflectionRegistry("rr", "rr") );

  // Add an always true event to pr
  EXPECT_TRUE(pr->addEvent("e1", "1"));

  // Now add pr to rr
  rr->addParameterCollection(pr);

  // And an event to rr based on pr
  EXPECT_TRUE(rr->addEvent("e1", "pr.e1"));
  EXPECT_TRUE(rr->emitEvents());
}

/*
 ____________________________
/  ________________________  \
| /  ____________________  \ |
| | < This is just silly > | |
| |  --------------------  | |
| |  \                     | |
| |   \                    | |
| |    \ >()_              | |
| \       (__)__ _         / |
|  ------------------------  |
|     \                      |
|      \  /\/\               |
|        \   /               |
|        |  0 >>             |
|        |___|               |
|  __((_<|   |               |
| (          |               |
| (__________)               |
|    |      |                |
|    |      |                |
\    /\     /\               /
 ----------------------------
  \                                  ,+*^^*+___+++_
   \                           ,*^^^^              )
    \                       _+*                     ^**+_
     \                    +^       _ _++*+_+++_,         )
              _+^^*+_    (     ,+*^ ^          \+_        )
             {       )  (    ,(    ,_+--+--,      ^)      ^\
            { (@)    } f   ,(  ,+-^ __*_*_  ^^\_   ^\       )
           {:;-/    (_+*-+^^^^^+*+*<_ _++_)_    )    )      /
          ( /  (    (        ,___    ^*+_+* )   <    <      \
           U _/     )    *--<  ) ^\-----++__)   )    )       )
            (      )  _(^)^^))  )  )\^^^^^))^*+/    /       /
          (      /  (_))_^)) )  )  ))^^^^^))^^^)__/     +^^
         (     ,/    (^))^))  )  ) ))^^^^^^^))^^)       _)
          *+__+*       (_))^)  ) ) ))^^^^^^))^^^^^)____*^
          \             \_)^)_)) ))^^^^^^^^^^))^^^^)
           (_             ^\__^^^^^^^^^^^^))^^^^^^^)
             ^\___            ^\__^^^^^^))^^^^^^^^)\\
                  ^^^^^\uuu/^^\uuu/^^^^\^\^\^\^\^\^\^\
                     ___) >____) >___   ^\_\_\_\_\_\_\)
                    ^^^//\\_^^//\\_^       ^(\_\_\_\)
                      ^^^ ^^ ^^^ ^
*/
TEST_F(ReflectionRegistryTest, Turducken)
{
  std::shared_ptr<controlit::ParameterReflection> duck( new controlit::ParameterReflection("rr", "duck") );
  std::shared_ptr<controlit::ReflectionRegistry> turkey( new controlit::ReflectionRegistry("rr", "turkey") );
  std::shared_ptr<controlit::ReflectionRegistry> platter( new controlit::ReflectionRegistry("rr", "platter") );

  EXPECT_TRUE(duck->addParameter("chicken", new std::string("chicken")));
  EXPECT_TRUE(turkey->addParameterCollection(duck));
  EXPECT_TRUE(platter->addParameterCollection(turkey));

  controlit::Parameter* param = platter->lookupParameter("turkey.duck.chicken");

  EXPECT_TRUE(param != NULL);
  EXPECT_TRUE(*(param->getString()) == "chicken");
}
