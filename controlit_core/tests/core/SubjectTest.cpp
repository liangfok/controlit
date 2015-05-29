#include <gtest/gtest.h>

#include <controlit/Subject.hpp>

class SubjectTest : public ::testing::Test
{
protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

/* ----------------------------------------------------------------------------
 *  Generic Subject functionality
 * --------------------------------------------------------------------------*/
//class MyListener : public controlit::UniqueObject
struct MyListener
{
public:
  MyListener() : val(0) {}

  typedef unsigned int SignalArgument_t;
  typedef void(SignalPrototype_t)(unsigned int const&);

  void update(SignalArgument_t const& ev)
  {
    val = ev;
  };

  unsigned int val;
};

class MySubject : public controlit::TypedSubject<MyListener>
{
public:
  MySubject() : n(0) {}
  void update()
  {
    n++;
    notifyListeners(n);
  }
  unsigned int n;
};

TEST_F(SubjectTest, NotifyListeners)
{
  MyListener l;
  MySubject s;

  //s.addListener(&l);
  MySubject::Connection_t connection = s.addListener(boost::bind(&MyListener::update, &l, _1));
  EXPECT_TRUE(l.val == s.n);
  s.update();
  EXPECT_TRUE(l.val == s.n);
  s.update();
  EXPECT_TRUE(l.val == s.n);
  connection.disconnect();
  s.update();
  EXPECT_TRUE(l.val == 2);
}
