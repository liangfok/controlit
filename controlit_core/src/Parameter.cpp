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

#include <controlit/Parameter.hpp>
#include <controlit/logging/RealTimeLogging.hpp>

namespace controlit {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

Parameter::Parameter(std::string const& name, ParameterType type, unsigned int flags)
  : name_(name),
    type_(type),
    flags_(flags)
{
    switch (type)
    {
        case PARAMETER_TYPE_VOID:
        case PARAMETER_TYPE_SIZE_T:
        case PARAMETER_TYPE_INTEGER:
        case PARAMETER_TYPE_UNSIGNED_INTEGER:
        case PARAMETER_TYPE_STRING:
        case PARAMETER_TYPE_REAL:
        case PARAMETER_TYPE_VECTOR:
        case PARAMETER_TYPE_MATRIX:
        case PARAMETER_TYPE_JOINT_STATE:
        case PARAMETER_TYPE_BINDING:
        case PARAMETER_TYPE_LIST:
          break;
        default:
          const_cast<ParameterType&>(type_) = PARAMETER_TYPE_VOID;
    }
}

Parameter::~Parameter()
{
}

size_t const* Parameter::getSizeT() const
{
    return 0;
}

unsigned int const* Parameter::getUnsignedInteger() const
{
    return 0;
}

int const* Parameter::getInteger() const
{
    return 0;
}

std::string const* Parameter::getString() const
{
    return 0;
}

double const* Parameter::getReal() const
{
    return 0;
}

Vector const* Parameter::getVector() const
{
    return 0;
}

Matrix const* Parameter::getMatrix() const
{
    return 0;
}

std::vector<std::string> const* Parameter::getList() const
{
    return 0;
}

JointState const* Parameter::getJointState() const
{
    return 0;
}

BindingConfig const* Parameter::getBindingConfig() const
{
    return 0;
}

std::string Parameter::getValueString() const
{
    return "UNDEFINED VALUE";
}

#if defined(__LP64__) || defined(_LP64)
bool Parameter::set(size_t const& value)
{
    CONTROLIT_ERROR << "Type Mismatch, got size_t, expected " + parameterTypeToString(type());
    return false;
}
#endif

bool Parameter::set(unsigned int const& value)
{
    CONTROLIT_ERROR << "Type Mismatch, got unsigned int, expected " + parameterTypeToString(type());
    return false;
}

bool Parameter::set(int const& value)
{
    CONTROLIT_ERROR << "Type Mismatch, got int, expected " + parameterTypeToString(type());
    return false;
}

bool Parameter::set(std::string const& value)
{
    CONTROLIT_ERROR << "Type Mismatch, got string, expected " + parameterTypeToString(type());
    return false;
}

bool Parameter::set(double const& value)
{
    CONTROLIT_ERROR << "Type Mismatch, got double, expected " + parameterTypeToString(type());
    return false;
}

bool Parameter::set(Vector const& value)
{
    CONTROLIT_ERROR << "Type Mismatch, got Vector, expected " + parameterTypeToString(type());
    return false;
}

bool Parameter::set(Matrix const& value)
{
    CONTROLIT_ERROR << "Type Mismatch, got Matrix, expected " + parameterTypeToString(type());
    return false;
}

bool Parameter::set(std::vector<std::string> const& value)
{
    CONTROLIT_ERROR << "Type Mismatch, got string vector, expected " + parameterTypeToString(type());
    return false;
}

bool Parameter::set(JointState const& value)
{
    CONTROLIT_ERROR << "Type Mismatch, got JointState, expected " + parameterTypeToString(type());
    return false;
}

bool Parameter::set(BindingConfig const& value)
{
    CONTROLIT_ERROR << "Type Mismatch, got BindingConfig, expected " + parameterTypeToString(type());
    return false;
}

void Parameter::dump(std::ostream& os, std::string const& prefix) const
{
    os << prefix << name_ << " : void\n";
}

std::string Parameter::parameterTypeToString(ParameterType paramType)
{
    switch(paramType)
    {
        case PARAMETER_TYPE_VOID:
            return "PARAMETER_TYPE_VOID";
        case PARAMETER_TYPE_STRING:
            return "PARAMETER_TYPE_STRING";
        case PARAMETER_TYPE_SIZE_T:
            return "PARAMETER_TYPE_SIZE_T";
        case PARAMETER_TYPE_UNSIGNED_INTEGER:
            return "PARAMETER_TYPE_UNSIGNED_INTEGER";
        case PARAMETER_TYPE_INTEGER:
            return "PARAMETER_TYPE_INTEGER";
        case PARAMETER_TYPE_REAL:
            return "PARAMETER_TYPE_REAL";
        case PARAMETER_TYPE_VECTOR:
            return "PARAMETER_TYPE_VECTOR";
        case PARAMETER_TYPE_MATRIX:
            return "PARAMETER_TYPE_MATRIX";
        case PARAMETER_TYPE_LIST:
            return "PARAMETER_TYPE_LIST";
  	    case PARAMETER_TYPE_JOINT_STATE:
            return "PARAMETER_TYPE_JOINT_STATE";
        case PARAMETER_TYPE_BINDING:
            return "PARAMETER_TYPE_BINDING";
        default:
            return "PARAMETER_TYPE_UNKNOWN";
    }
}

/* ****************************************************************************
 *  SizeTParameter
 * ***************************************************************************/
#if defined(__LP64__) || defined(_LP64)

SizeTParameter::SizeTParameter(std::string const& name, unsigned int flags, size_t* instance)
    : Parameter(name, PARAMETER_TYPE_SIZE_T, flags), sizeT_(instance)
{
}

SizeTParameter::~SizeTParameter()
{
    PRINT_DEBUG_STATEMENT("Method Called! ownsData = " << ownsData())
  
    if (ownsData())
    {
        PRINT_DEBUG_STATEMENT("Deleting local pointer.")
        delete sizeT_;
    }
  
    PRINT_DEBUG_STATEMENT("Done method Call.")
}

bool SizeTParameter::set(size_t const& sizeT)
{
    // PRINT_DEBUG_STATEMENT("Method Called!")
  
    if (readOnly())
    {
        CONTROLIT_ERROR << "Attempted to set read-only parameter.";
        return false;
    } 
  
    *sizeT_ = sizeT;
    notifyListeners(*this);
    return true;
}


void SizeTParameter::dump(std::ostream& os, std::string const& prefix) const
{
    os << prefix << name_ << " : size_t = " << *sizeT_ << "\n";
}

#endif

/* ****************************************************************************
 *  UnsignedIntegerParameter
 * ***************************************************************************/
UnsignedIntegerParameter::UnsignedIntegerParameter(std::string const& name, 
    unsigned int flags, unsigned int* unsignedInteger)
    : Parameter(name, PARAMETER_TYPE_UNSIGNED_INTEGER, flags),
      unsignedInteger_(unsignedInteger)
{
}

UnsignedIntegerParameter::~UnsignedIntegerParameter()
{
    PRINT_DEBUG_STATEMENT("Method Called! ownsData = " << ownsData())
  
    if (ownsData())
    {
        PRINT_DEBUG_STATEMENT("Deleting local pointer.")
        delete unsignedInteger_;
    }
  
    PRINT_DEBUG_STATEMENT("Done method Call.")
}

bool UnsignedIntegerParameter::set(unsigned int const & unsignedInteger)
{
    // PRINT_DEBUG_STATEMENT("Method Called!")

    if (readOnly())
    {
        CONTROLIT_ERROR << "Attempted to set read-only parameter.";
        return false;
    } 
    
    *unsignedInteger_ = unsignedInteger;
    notifyListeners(*this);
    return true;
}


void UnsignedIntegerParameter::dump(std::ostream& os, std::string const& prefix) const
{
    os << prefix << name_ << " : unsigned integer = " << *unsignedInteger_ << "\n";
}

/* ****************************************************************************
 *  IntegerParameter
 * ***************************************************************************/
IntegerParameter::IntegerParameter(std::string const & name, unsigned int flags, int * integer)
  : Parameter(name, PARAMETER_TYPE_INTEGER, flags),
    integer_(integer)
{
}

IntegerParameter::~IntegerParameter()
{
    PRINT_DEBUG_STATEMENT("Method Called! ownsData = " << ownsData())
  
    if (ownsData())
    {
        PRINT_DEBUG_STATEMENT("Deleting local pointer.")
        delete integer_;
    }
  
    PRINT_DEBUG_STATEMENT("Done method Call.")
}

bool IntegerParameter::set(int const& integer)
{  
    if (readOnly())
    {
        CONTROLIT_ERROR << "Attempted to set read-only parameter.";
        return false;
    } 

    *integer_ = integer;
    notifyListeners(*this);
    return true;
}

void IntegerParameter::dump(std::ostream& os, std::string const& prefix) const
{
    os << prefix << name_ << " : integer = " << *integer_ << "\n";
}



/* ****************************************************************************
 *  StringParameter
 * ***************************************************************************/
StringParameter::StringParameter(std::string const& name, unsigned int flags, std::string* instance)
    : Parameter(name, PARAMETER_TYPE_STRING, flags), string_(instance)
{
}

StringParameter::~StringParameter()
{
    PRINT_DEBUG_STATEMENT("Method Called! ownsData = " << ownsData())
  
    if (ownsData())
    {
        PRINT_DEBUG_STATEMENT("Deleting local pointer.")
        delete string_;
    }
  
    PRINT_DEBUG_STATEMENT("Done method Call.")
}

bool StringParameter::set(std::string const& value)
{
    // PRINT_DEBUG_STATEMENT("Method Called!")
  
    if (readOnly())
    {
        CONTROLIT_ERROR << "Attempted to set read-only parameter.";
        return false;
    } 
  
    *string_ = value;
    notifyListeners(*this);
    return true;
}

void StringParameter::dump(std::ostream& os, std::string const& prefix) const
{
    os << prefix << name_ << " : string = " << *string_ << "\n";
}


/* ****************************************************************************
 *  RealParameter
 * ***************************************************************************/
RealParameter::RealParameter(std::string const& name, unsigned int flags, double * real)
    : Parameter(name, PARAMETER_TYPE_REAL, flags),
      real_(real)
{
}

RealParameter::~RealParameter()
{
    PRINT_DEBUG_STATEMENT("Method Called! ownsData = " << ownsData())
  
    if (ownsData())
    {
        PRINT_DEBUG_STATEMENT("Deleting local pointer.")
        delete real_;
    }
  
    PRINT_DEBUG_STATEMENT("Done method Call.")
}

bool RealParameter::set(double const & real)
{
    if (readOnly())
    {
        CONTROLIT_ERROR << "Attempted to set read-only parameter.";
        return false;
    } 

    *real_ = real;
    notifyListeners(*this);
    return true;
}

void RealParameter::dump(std::ostream& os, std::string const& prefix) const
{
    os << prefix << name_ << " : real = " << *real_ << "\n";
}

/* ****************************************************************************
 *  VectorParameter
 * ***************************************************************************/
VectorParameter::VectorParameter(std::string const& name, unsigned int flags, Vector * vector)
    : Parameter(name, PARAMETER_TYPE_VECTOR, flags),
      vector_(vector)
{
}

VectorParameter::~VectorParameter()
{
    PRINT_DEBUG_STATEMENT("Method Called! ownsData = " << ownsData())
  
    if (ownsData())
    {
       PRINT_DEBUG_STATEMENT("Deleting local pointer.")
       delete vector_;
    }
  
    PRINT_DEBUG_STATEMENT("Done method Call.")
}

bool VectorParameter::set(Vector const& vector)
{
    if (readOnly())
    {
        CONTROLIT_ERROR << "Attempted to set read-only parameter.";
        return false;
    } 

    *vector_ = vector;
    notifyListeners(*this);
    return true;
}

std::string VectorParameter::getValueString() const
{
    std::stringstream buff;
  
    buff << "[";
  
    for (int ii = 0; ii < vector_->size(); ii++)
    {
        buff << (*vector_)[ii];
        if (ii < vector_->size() - 1)
            buff << ", ";
    }
  
    buff << "]";
    return buff.str();
}

void VectorParameter::dump(std::ostream& os, std::string const& prefix) const
{
    /*os << prefix << name_ << " : vector =\n"
    << prefix << "  " << pretty_string(*vector_) << "\n";*/
}



/* ****************************************************************************
 *  MatrixParameter
 * ***************************************************************************/
MatrixParameter::MatrixParameter(std::string const& name, unsigned int flags, Matrix* matrix)
    : Parameter(name, PARAMETER_TYPE_MATRIX, flags),
      matrix_(matrix)
{
}

MatrixParameter::~MatrixParameter()
{
    PRINT_DEBUG_STATEMENT("Method Called! ownsData = " << ownsData())
  
    if (ownsData())
    {
        PRINT_DEBUG_STATEMENT("Deleting local pointer.")
        delete matrix_;
    }
  
    PRINT_DEBUG_STATEMENT("Done method Call.")
}

bool MatrixParameter::set(Matrix const& matrix)
{
    // PRINT_DEBUG_STATEMENT("Method Called!")
  
    if (readOnly())
    {
        CONTROLIT_ERROR << "Attempted to set read-only parameter.";
        return false;
    } 
  
    *matrix_ = matrix;
    notifyListeners(*this);
    return true;
}

std::string MatrixParameter::getValueString() const
{
    std::stringstream buff;
    buff << "[";
  
    for (int ii = 0; ii < matrix_->rows(); ii++)
    {
        for (int jj = 0; jj < matrix_->cols(); jj++)
        {
            buff << (*matrix_)(ii, jj);
            if (jj < matrix_->cols() - 1)
                buff << ", ";
        }
    
        if (ii < matrix_->rows() - 1)
            buff << "\n";
    }
  
    buff << "]";
    return buff.str();
}

void MatrixParameter::dump(std::ostream& os, std::string const& prefix) const
{
    //os << prefix << name_ << " : matrix =\n"
    //   << pretty_string(*matrix_, prefix + "  ") << "\n";
}

/* ****************************************************************************
 *  ListParameter
 * ***************************************************************************/
ListParameter::ListParameter(std::string const& name,
                unsigned int flags, std::vector<std::string> * list)
    : Parameter(name, PARAMETER_TYPE_LIST, flags),
      list_(list)
{
}

ListParameter::~ListParameter()
{
    PRINT_DEBUG_STATEMENT("Method Called! ownsData = " << ownsData())
  
    if (ownsData())
    {
        PRINT_DEBUG_STATEMENT("Deleting local pointer.")
        delete list_;
    }
  
    PRINT_DEBUG_STATEMENT("Done method Call.")
}

bool ListParameter::set(std::vector<std::string> const& list)
{
    if (readOnly())
    {
        CONTROLIT_ERROR << "Attempted to set read-only parameter.";
        return false;
    } 

    *list_ = list;
    notifyListeners(*this);
    return true;
}

std::string ListParameter::getValueString() const
{
    std::stringstream buff;
    buff << "[";
  
    for (unsigned int ii = 0; ii < list_->size(); ii++)
    {
        buff << (*list_)[ii];
        if (ii < list_->size() - 1)
            buff << ", ";
    }
  
    buff << "]";
    return buff.str();
}

void ListParameter::dump(std::ostream& os, std::string const& prefix) const
{
    //os << prefix << name_ << " : matrix =\n"
    //   << pretty_string(*matrix_, prefix + "  ") << "\n";
}


/* ****************************************************************************
 *  JointStateParameter
 * ***************************************************************************/
JointStateParameter::JointStateParameter(std::string const& name,
                unsigned int flags, JointState* jointState)
    : Parameter(name, PARAMETER_TYPE_JOINT_STATE, flags),
      jointState_(jointState)
{
}

JointStateParameter::~JointStateParameter()
{
    PRINT_DEBUG_STATEMENT("Method Called! ownsData = " << ownsData())
  
    if (ownsData())
    {
        PRINT_DEBUG_STATEMENT("Deleting local pointer.")
        delete jointState_;
    }
  
    PRINT_DEBUG_STATEMENT("Done method Call.")
}

bool JointStateParameter::set(JointState const& jointState)
{
    if (readOnly())
    {
        CONTROLIT_ERROR << "Attempted to set read-only parameter.";
        return false;
    } 
  
    *jointState_ = jointState;
    notifyListeners(*this);
    return true;
}

std::string JointStateParameter::getValueString() const
{
    std::stringstream buff;
  
    buff << "\n        Header:\n";
    buff << "          seq: " << jointState_->header.seq << "\n";
    buff << "          stamp: " << ros::Time(jointState_->header.stamp.sec, jointState_->header.stamp.nsec).toSec() << "\n";
    buff << "          frame_id: " << jointState_->header.frame_id << "\n";
  
    size_t numDOFs = jointState_->name.size();
  
    buff << "        Name: [";
  
    for (size_t ii = 0; ii < numDOFs; ii++)
    {
        buff << jointState_->name[ii];
        if (ii < numDOFs - 1)
            buff << ", ";
    }
  
    buff << "]\n";
  
    numDOFs = jointState_->position.size();
  
    buff << "        Position: [";
  
    for (size_t ii = 0; ii < numDOFs; ii++)
    {
        buff << jointState_->position[ii];
        if (ii < numDOFs - 1)
            buff << ", ";
    }
  
    buff << "]\n";
  
    numDOFs = jointState_->velocity.size();
  
    buff << "        Velocity: [";
  
    for (size_t ii = 0; ii < numDOFs; ii++)
    {
        buff << jointState_->velocity[ii];
        if (ii < numDOFs - 1)
            buff << ", ";
    }
  
    buff << "]\n";
  
  
    numDOFs = jointState_->effort.size();
  
    buff << "        Effort: [";
  
    for (size_t ii = 0; ii < numDOFs; ii++)
    {
        buff << jointState_->effort[ii];
        if (ii < numDOFs - 1)
            buff << ", ";
    }
  
    buff << "]";
  
    return buff.str();
}

void JointStateParameter::dump(std::ostream& os, std::string const& prefix) const
{
    //os << prefix << name_ << " : jointState =\n"
    //   << pretty_string(*jointState_, prefix + "  ") << "\n";
}

/* ****************************************************************************
 *  BindingConfigParameter
 * ***************************************************************************/
BindingConfigParameter::BindingConfigParameter(std::string const& name,
                unsigned int flags, BindingConfig* binding)
    : Parameter(name, PARAMETER_TYPE_BINDING, flags),
      binding_(binding)
{
}

BindingConfigParameter::~BindingConfigParameter()
{
    PRINT_DEBUG_STATEMENT("Method Called! ownsData = " << ownsData())
  
    if (ownsData())
    {
        PRINT_DEBUG_STATEMENT("Deleting local pointer.")
        delete binding_;
    }
  
    PRINT_DEBUG_STATEMENT("Done method Call.")
}

bool BindingConfigParameter::set(BindingConfig const& binding)
{
    if (readOnly())
    {
        CONTROLIT_ERROR << "Attempted to set read-only parameter.";
        return false;
    } 

    *binding_ = binding;
    notifyListeners(*this);
    return true;
}

void BindingConfigParameter::dump(std::ostream& os, std::string const& prefix) const
{
}

} // namespace controlit
