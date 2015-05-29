/*
 * Copyright (C) 2015 The University of Texas at Austin and the
 * Institute of Human Machine Cognition. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 2.1 of
 * the License, or (at your option) any later version. See
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#ifndef __CONTROLIT_CORE_PARAMETER_HPP__
#define __CONTROLIT_CORE_PARAMETER_HPP__

#ifdef __GNUC__
# define DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
# define DEPRECATED(func) __declspec(deprecated) func
#else
# pragma message("WARNING: You need to implement DEPRECATED for this compiler")
# define DEPRECATED(func) func
#endif

#include <sstream>
#include <map>
#include <vector>

#include <controlit/BindingConfig.hpp>
#include <controlit/ParameterListener.hpp>
#include <controlit/Subject.hpp>
#include <sensor_msgs/JointState.h>

#include <controlit/addons/eigen/LinearAlgebra.hpp>

using sensor_msgs::JointState;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector;

namespace controlit {

/**
   Enumeration type for task parameter types.
*/
typedef enum
{
    PARAMETER_TYPE_VOID,             // no data (e.g. invalid type code)
    PARAMETER_TYPE_STRING,           // Mapped to std::string
    PARAMETER_TYPE_SIZE_T,           // Mapped to size_t
    PARAMETER_TYPE_UNSIGNED_INTEGER, // Mapped to unsigned int
    PARAMETER_TYPE_INTEGER,          // Mapped to int
    PARAMETER_TYPE_REAL,             // Mapped to double
    PARAMETER_TYPE_VECTOR,           // Mapped to Vector
    PARAMETER_TYPE_MATRIX,           // Mapped to Matrix
    PARAMETER_TYPE_LIST,  		   // Mapped to std::vector<std::string>
    PARAMETER_TYPE_JOINT_STATE,      // Mapped to sensor_msgs::JointState
    PARAMETER_TYPE_BINDING           // Mapped to controlit::BindingConfig
} ParameterType;


// Forward declarations
// class ParameterReflection;

/**
 * The parant class of all parameters in ControlIt!.
 */
class Parameter : public controlit::TypedSubject<ParameterListener>
{
public:

    // Defines flags that denote certain parameter properties.
    struct Flag
    {
        static const unsigned int Default  =  0x00;
        static const unsigned int ReadOnly =  0x01;
        static const unsigned int OwnsData =  0x02;
    };

    /*!
     * The constructor.
     *
     * \param[in] name The parameter's name.
     * \param[in] type The parameter's type.
     * \param[in] flags Flags indicating special properties of the parameter.
     */
    Parameter(const std::string & name, ParameterType type, unsigned int flags);

    /*!
     * The destructor.
     */
    virtual ~Parameter();

    /*!
     * Accessors to the underlying parameter data.
     *
     * TODO: Replace these with templates.
     */
    virtual size_t const*                   getSizeT() const;
    virtual unsigned int const*             getUnsignedInteger() const;
    virtual int const*                      getInteger() const;
    virtual std::string const*              getString() const;
    virtual double const*                   getReal() const;
    virtual Vector const*                   getVector() const;
    virtual Matrix const*                   getMatrix() const;
    virtual std::vector<std::string> const* getList() const;
    virtual JointState const*               getJointState() const;
    virtual BindingConfig const*            getBindingConfig() const;

    /*!
     * Gets a string representation of this parameter's value.
     *
     * \return a string representation of this parameter's value.
     */
    virtual std::string getValueString() const;

    /*!
     * Mutators.
     *
     * TODO: Replace these with templates.
     */
#if defined(__LP64__) || defined(_LP64)
    virtual bool set(size_t const& value);
#endif
    virtual bool set(unsigned int const& value);
    virtual bool set(int const& value);
    virtual bool set(std::string const& value);
    virtual bool set(double const& value);
    virtual bool set(Vector const& value);
    virtual bool set(Matrix const& value);
    virtual bool set(std::vector<std::string> const& value);
    virtual bool set(JointState const& value);
    virtual bool set(BindingConfig const& value);

    /*!
     * Produces a string representation of this class.
     *
     * \param[in] os The output stream to which to write the string.
     * \param[in] prefix What to add to the beginning each line of the string.
     */
    virtual void dump(std::ostream& os, std::string const& prefix) const;

    /*!
     * \return The name of this parameter.
     */
    inline std::string name() const {return name_;}

    /*!
     * \return The type of this parameter.
     */
    inline ParameterType type() const {return type_;}

    /*!
     * Checks whether this parameter is of a certain type.
     *
     * \param[in] type The specified type.
     * \return true if this parameter is of the specified type.
     */
    inline bool isType(ParameterType type) const {return (type_ == type);}

    /*!
     * \return Whether the OwnsData flag is set.
     */
    inline bool ownsData() const {return (flags_ & Parameter::Flag::OwnsData);}

    /*!
     * \return Whether the ReadOnly flag is set.
     */
    inline bool readOnly() const {return (flags_ & Parameter::Flag::ReadOnly);}

    /*!
     * Returns a string representation of a parameter type.
     * This is useful for generating meaningful debug messages, e.g.,
     * instead of 0 the message will have "PARAMETER_TYPE_STRING".
     *
     * \param paramType The parameter type.
     * \return A string representation of the parameter type.
     */
    static std::string parameterTypeToString(ParameterType paramType);

protected:

    /*!
     * The name of the parameter.
     */
    const std::string name_;

    /*!
     * The type of the parameter.
     */
    ParameterType const type_;

    /*!
     * Flags indicating special properties of the parameter.
     */
    unsigned int const flags_;
};

// TODO: Change to a shared_ptr!
typedef std::map<std::string, Parameter*> ParameterMap;

template<class T>
struct ParameterFactory
{
    static Parameter * create(std::string const& name, unsigned int flags, T* instance)
    {
        // Generic implementation not defined
        return NULL;
    }
};

template<class T>
struct ParameterAccessor
{
    static T const * get(Parameter const * p)
    {
        // Generic implementation not defined
        return NULL;
    }
  
    static std::string type(Parameter const * p)
    {
        return std::string("_UNKNOWN_TYPE_");
    }
};

/* ****************************************************************************
 *  SizeTParameter
 * ***************************************************************************/
#if defined(__LP64__) || defined(_LP64)

class SizeTParameter : public Parameter
{
public:
    SizeTParameter(std::string const& name,
                   unsigned int flags,
                   size_t* instance);
    virtual ~SizeTParameter();
    virtual size_t const* getSizeT() const { return sizeT_; }
    virtual bool set(size_t const& sizeT);
    virtual void dump(std::ostream& os, std::string const& prefix) const;
    virtual std::string getValueString() const { return std::to_string(*sizeT_); }
protected:
    size_t * sizeT_;
};

template<>
struct ParameterFactory<size_t>
{
    static Parameter* create(std::string const& name, unsigned int flags, size_t * instance)
    {
        return new controlit::SizeTParameter(name, flags, instance);
    }
};

template<>
struct ParameterAccessor<size_t>
{
    static size_t const* get(Parameter const * p)
    {
        return p->getSizeT();
    }
  
    static std::string type(Parameter const * p)
    {
        return std::string("size_t");
    }
};
#endif

/* ****************************************************************************
 *  UnsignedIntegerParameter
 * ***************************************************************************/
class UnsignedIntegerParameter : public Parameter
{
public:
    UnsignedIntegerParameter(std::string const& name,
                             unsigned int flags,
                             unsigned int* instance);
    virtual ~UnsignedIntegerParameter();
    virtual unsigned int const* getUnsignedInteger() const { return unsignedInteger_; }
    virtual bool set(unsigned int const& unsignedInteger);
    virtual void dump(std::ostream& os, std::string const& prefix) const;
    virtual std::string getValueString() const { return std::to_string(*unsignedInteger_); }
protected:
    unsigned int * unsignedInteger_;
};

template<>
struct ParameterFactory<unsigned int>
{
    static Parameter* create(std::string const & name, unsigned int flags, unsigned int* instance)
    {
        return new controlit::UnsignedIntegerParameter(name, flags, instance);
    }
};

template<>
struct ParameterAccessor<unsigned int>
{
    static unsigned int const* get(Parameter const* p)
    {
        return p->getUnsignedInteger();
    }
  
    static std::string type(Parameter const* p)
    {
        return std::string("unsigned int");
    }
};

/* ****************************************************************************
 *  IntegerParameter
 * ***************************************************************************/
class IntegerParameter : public Parameter
{
public:
    IntegerParameter(std::string const& name,
                     unsigned int flags,
                     int* instance);
    virtual ~IntegerParameter();
    virtual int const* getInteger() const { return integer_; }
    virtual bool set(int const& integer);
    virtual void dump(std::ostream& os, std::string const& prefix) const;
    virtual std::string getValueString() const { return std::to_string(*integer_); }
protected:
    int* integer_;
};

template<>
struct ParameterFactory<int>
{
    static Parameter * create(std::string const & name, unsigned int flags, int * instance)
    {
        return new controlit::IntegerParameter(name, flags, instance);
    }
};

template<>
struct ParameterAccessor<int>
{
    static int const* get(Parameter const* p)
    {
        return p->getInteger();
    }
  
    static std::string type(Parameter const* p)
    {
        return std::string("int");
    }
};



/* ****************************************************************************
 *  StringParameter
 * ***************************************************************************/
class StringParameter : public Parameter
{
public:
    StringParameter(std::string const& name,
                    unsigned int flags,
                    std::string* instance);
    virtual ~StringParameter();
    virtual std::string const* getString() const { return string_; }
    virtual bool set(std::string const& value);
    virtual void dump(std::ostream& os, std::string const& prefix) const;
  
    virtual std::string getValueString() const { return *string_; }
protected:
    std::string * string_;
};

template<>
struct ParameterFactory<std::string>
{
    static Parameter* create(std::string const& name, unsigned int flags, std::string* instance)
    {
        return new controlit::StringParameter(name, flags, instance);
    }
};

template<>
struct ParameterAccessor<std::string>
{
    static std::string const* get(Parameter const* p)
    {
        return p->getString();
    }
  
    static std::string type(Parameter const* p)
    {
        return std::string("string");
    }
};



/* ****************************************************************************
 *  RealParameter
 * ***************************************************************************/
class RealParameter : public Parameter
{
public:
    RealParameter(std::string const& name,
                  unsigned int flags,
                  double* real);
    virtual ~RealParameter();
    virtual double const* getReal() const { return real_; }
    virtual bool set(double const& real);
    virtual void dump(std::ostream& os, std::string const& prefix) const;
    virtual std::string getValueString() const { return std::to_string(*real_); }
protected:
    double * real_;
};

template<>
struct ParameterFactory<double>
{
    static Parameter* create(std::string const& name, unsigned int flags, double* instance)
    {
        return new controlit::RealParameter(name, flags, instance);
    }
};

template<>
struct ParameterAccessor<double>
{
    static double const* get(Parameter const * p)
    {
        return p->getReal();
    }
  
    static std::string type(Parameter const * p)
    {
        return std::string("double");
    }
};



/* ****************************************************************************
 *  VectorParameter
 * ***************************************************************************/
class VectorParameter : public Parameter
{
public:
    VectorParameter(std::string const& name,
                    unsigned int flags,
                    Vector* vector);
    virtual ~VectorParameter();
    virtual Vector const* getVector() const { return vector_; }
    virtual bool set(Vector const& vector);
    virtual void dump(std::ostream& os, std::string const& prefix) const;
    virtual std::string getValueString() const;
  
protected:
    Vector * vector_;
};

template<>
struct ParameterFactory<Vector>
{
    static Parameter* create(std::string const& name, unsigned int flags, Vector* instance)
    {
        return new VectorParameter(name, flags, instance);
    }
};

template<>
struct ParameterAccessor<Vector>
{
    static Vector const* get(Parameter const* p)
    {
        return p->getVector();
    }
  
    static std::string type(Parameter const* p)
    {
        return std::string("vector");
    }
};



/* ****************************************************************************
 *  MatrixParameter
 * ***************************************************************************/
class MatrixParameter : public Parameter
{
public:
    MatrixParameter(std::string const& name,
                    unsigned int flags,
                    Matrix* matrix);
    virtual ~MatrixParameter();
    virtual Matrix const* getMatrix() const { return matrix_; }
    virtual bool set(Matrix const& matrix);
    virtual void dump(std::ostream& os, std::string const& prefix) const;
    virtual std::string getValueString() const;

protected:
    Matrix* matrix_;
};

template<>
struct ParameterFactory<Matrix>
{
  static Parameter* create(std::string const& name, unsigned int flags, Matrix* instance)
  {
    return new MatrixParameter(name, flags, instance);
  }
};

template<>
struct ParameterAccessor<Matrix>
{
  static Matrix const* get(Parameter const* p)
  {
    return p->getMatrix();
  }

  static std::string type(Parameter const* p)
  {
    return std::string("matrix");
  }
};

/* ****************************************************************************
 *  ListParameter
 * ***************************************************************************/
class ListParameter : public Parameter
{
public:
  ListParameter(std::string const& name,
                  unsigned int flags,
                  std::vector<std::string> * list);
  virtual ~ListParameter();
  virtual std::vector<std::string> const* getList() const { return list_; }
  virtual bool set(std::vector<std::string> const& list);
  virtual void dump(std::ostream& os, std::string const& prefix) const;
  virtual std::string getValueString() const;

protected:
  std::vector<std::string> * list_;
};

template<>
struct ParameterFactory< std::vector<std::string> >
{
  static Parameter* create(std::string const& name, unsigned int flags, std::vector<std::string> * instance)
  {
    return new controlit::ListParameter(name, flags, instance);
  }
};

template<>
struct ParameterAccessor< std::vector<std::string> >
{
  static std::vector<std::string> const* get(Parameter const* p)
  {
    return p->getList();
  }

  static std::string type(Parameter const* p)
  {
    return std::string("list");
  }
};

/* ****************************************************************************
 *  JointState Parameter
 * ***************************************************************************/
class JointStateParameter : public Parameter
{
public:
  JointStateParameter(std::string const& name,
                  unsigned int flags,
                  JointState* jointState);
  virtual ~JointStateParameter();
  virtual JointState const* getJointState() const
  {
    return jointState_;
  }
  virtual bool set(JointState const& jointState);
  virtual std::string getValueString() const;
  virtual void dump(std::ostream& os, std::string const& prefix) const;
protected:
  JointState * jointState_;
};

template<>
struct ParameterFactory<JointState>
{
  static Parameter * create(std::string const& name, unsigned int flags, JointState * instance)
  {
    return new controlit::JointStateParameter(name, flags, instance);
  }
};

template<>
struct ParameterAccessor<JointState>
{
  static JointState const * get(Parameter const * p)
  {
    return p->getJointState();
  }

  static std::string type(Parameter const * p)
  {
    return std::string("JointState");
  }
};



/* ****************************************************************************
 *  BindingConfigParameter
 * ***************************************************************************/
class BindingConfigParameter : public Parameter
{
public:
    BindingConfigParameter(std::string const& name,
                           unsigned int flags,
                           BindingConfig* binding);
    virtual ~BindingConfigParameter();
    virtual BindingConfig const* getBindingConfig() const { return binding_; }
    virtual bool set(BindingConfig const& binding);
    virtual void dump(std::ostream& os, std::string const& prefix) const;
    virtual std::string getValueString() const { return binding_->toString(); }
  
protected:
    BindingConfig* binding_;
};

template<>
struct ParameterFactory<controlit::BindingConfig>
{
    static Parameter* create(std::string const& name, unsigned int flags,
        controlit::BindingConfig* instance)
    {
        return new controlit::BindingConfigParameter(name, flags, instance);
    }
};

template<>
struct ParameterAccessor<controlit::BindingConfig>
{
    static controlit::BindingConfig const* get(Parameter const* p)
    {
        return p->getBindingConfig();
    }
  
    static std::string type(Parameter const* p)
    {
        return std::string("binding");
    }
};


} // namespace controlit

#endif // __CONTROLIT_CORE_PARAMETER_HPP__
