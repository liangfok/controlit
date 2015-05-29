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

#ifndef __CONTROLIT_CORE_PARAMETER_REFLECTION_HPP__
#define __CONTROLIT_CORE_PARAMETER_REFLECTION_HPP__

#include <controlit/logging/RealTimeLogging.hpp>
#include "ros/ros.h"

#include <controlit/Parameter.hpp>
#include <controlit/Subject.hpp>
#include <controlit/EventListener.hpp>

// For current version number on ROS Indigo, see: /opt/ros/indigo/include/ros/common.h
#if ROS_VERSION_MINIMUM(1, 11, 0) // if current ros version is >= 1.11.0
    // Indigo code
    #include <muParser.h>
#else
    // Hydro code
    #include <muParser/muParser.h>
#endif

#include <controlit/addons/eigen/LinearAlgebra.hpp>

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;

// Currently, many classes have ParameterReflection in their
// inheritance path. ParameterReflection has a type and instance name
// parameter that is defined by child classes. These overloads of the
// DRC_XXX macros use that information in custom fields.
#define CONTROLIT_PR_DEBUG  CONTROLIT_DEBUG ("controlit_instance_name", getInstanceName())("controlit_type", getTypeName())
#define CONTROLIT_PR_INFO   CONTROLIT_INFO ("controlit_instance_name", getInstanceName())("controlit_type", getTypeName())
#define CONTROLIT_PR_WARN   CONTROLIT_WARN ("controlit_instance_name", getInstanceName())("controlit_type", getTypeName())
#define CONTROLIT_PR_ERROR  CONTROLIT_ERROR ("controlit_instance_name", getInstanceName())("controlit_type", getTypeName())
#define CONTROLIT_PR_FATAL  DRC_FATAL ("controlit_instance_name", getInstanceName())("controlit_type", getTypeName())

#define CONTROLIT_PR_DEBUG_RT  CONTROLIT_DEBUG_RT ("controlit_instance_name", getInstanceName())("controlit_type", getTypeName())
#define CONTROLIT_PR_INFO_RT   CONTROLIT_INFO_RT ("controlit_instance_name", getInstanceName())("controlit_type", getTypeName())
#define CONTROLIT_PR_WARN_RT   CONTROLIT_WARN_RT ("controlit_instance_name", getInstanceName())("controlit_type", getTypeName())
#define CONTROLIT_PR_ERROR_RT  CONTROLIT_ERROR_RT ("controlit_instance_name", getInstanceName())("controlit_type", getTypeName())
#define CONTROLIT_PR_FATAL_RT  DRC_FATAL_RT ("controlit_instance_name", getInstanceName())("controlit_type", getTypeName())


namespace controlit {

// Container for event information
struct Event
{
    std::string name;
    mu::Parser condition;
    bool enabled;
};

/*!
 * This is a base for classes that reflect (some of) their parameters. It manages
 * a table of Parameter objects.
 */
class ParameterReflection : public controlit::TypedSubject<EventListener>
{
protected:
    std::vector<Event> events;

public:
    typedef std::list<Parameter const *> ParameterList_t;

    /*!
     * The constructor.
     *
     * \param[in] type_name A description of the type of this object,
     * e.g., "Task" or "Constraint"
     * \param[in] instance_name A description of the instance of this object,
     * e.g., "MyTask" or "FooConstraint".
     */
    ParameterReflection(std::string const & type_name, std::string const & instance_name);

    /*!
     * The destructor.
     */
    virtual ~ParameterReflection();

    /*!
     * Returns the type name of this class.
     */
    inline std::string const & getTypeName() const {return typeName;}

    /*!
     * Returns the instance name of this class.
     */
    inline std::string const & getInstanceName() const {return instanceName;}

    /*!
     * Returns the entire set of parameters.
     */
    ParameterMap const & getParameterTable() const {return parameterMap;}

    /*!
     * Gets a string representation of the parameters stored in this reflection registry.
     *
     * \param[out] keys A reference to where the names of the parameters should be stored.
     * \param[out] values A reference to where the values of the parameters should be stored.
     */
    virtual bool getParameters(std::vector<std::string> & keys, std::vector<std::string> & values) const;

    // Public event interface

    /*!
     * Add an event.
     *
     * \param name The name of the event.
     * \param expr The expression that triggers the event.
     */
    virtual bool addEvent(std::string const & name, std::string const & expr);

    /*!
     * \return The events added to this object.
     */
    std::vector<Event> const & getEvents() const {return events;}

//     /** Default implementation always returns success. */
// #if defined(__LP64__) || defined(_LP64)
//     virtual bool check(size_t const * param, size_t value) const;
// #endif
//     virtual bool check(unsigned int const * param, unsigned int value) const;
//     virtual bool check(int const * param, int value) const;
//     virtual bool check(std::string const * param, std::string const& value) const;
//     virtual bool check(double const * param, double value) const;
//     virtual bool check(Vector const * param, Vector const & value) const;
//     virtual bool check(Matrix const * param, Matrix const & value) const;
//     virtual bool check(std::vector<std::string> const * param, std::vector<std::string> const & value) const;
//     virtual bool check(BindingConfig const * param, BindingConfig const & value) const;

    /*!
     * Creates a ROS input binding with default parameters.
     *
     * \param paramName The name of the parameter and ROS topic.
     * \param dataType The ROS message data type.
     * \return The newly created Binding Config.
     */
    BindingConfig * createROSInputBinding(std::string paramName, std::string dataType);

    /*!
     * Creates a ROS output binding with default parameters.
     *
     * \param paramName The name of the parameter and ROS topic.
     * \param dataType The ROS message data type.
     * \param latched Whether to latch the ROS topic, default 0.
     * \param publishRate The publish rate in Hz, default 100.
     * \param queueSize the queue size in number of messages, default 1.
     * \return The newly created Binding Config.
     */
    BindingConfig * createROSOutputBinding(std::string paramName, std::string dataType, std::string latched = "0", 
        std::string publishRate = "100", std::string queueSize = "1");


    /*!
     * Determines whether a particular parameter exists.
     *
     * \param[in] name The name of the parameter
     * \return Whether a parameter of a particular name exists.
     */
    virtual bool hasParameter(std::string const& name) const;

    bool hasBinding(std::string const& name) const;

    /*!
     * Adds a parameter.
     *
     * \param[in] instance The underlying parameter variable.
     * \param[in] flags The parameter flags.
     * \return A pointer to the newly created parameter.
     */
    template<class T>
    Parameter * addParameter(T * instance, unsigned int flags = Parameter::Flag::Default)
    {
        return declareParameter<T>(instance->name(), instance, flags | Parameter::Flag::OwnsData);
    }

    /*!
     * Adds a parameter.
     *
     * \param[in] name The name of the parameter.
     * \param[in] instance The underlying parameter variable.
     * \param[in] flags The parameter flags.
     * \return A pointer to the newly created parameter.
     */
    template<class T>
    Parameter * addParameter(std::string const& name, T * instance, unsigned int flags = Parameter::Flag::Default)
    {
        return declareParameter<T>(name, instance, flags | Parameter::Flag::OwnsData);
    }

    /*!
     * Obtains a list of parameters of a particular type.
     *
     * \param[in] type The type of the parameter.
     * \return the parameters of the specified type.
     */
    virtual ParameterList_t lookupParameters(ParameterType type) const;

    /*!
     * Obtains a particular parameter.
     *
     * \param[in] name The name of the parameter.
     * \return A pointer to the specified parameter or nullptr if no such parameter exists.
     */
    virtual Parameter * lookupParameter(std::string const & name);

    /*!
     * Obtains a particular parameter.
     *
     * \param[in] name The name of the parameter.
     * \return A pointer to the specified parameter or nullptr if no such parameter exists.
     */
    virtual Parameter const * lookupParameter(std::string const & name) const;

    /*!
     * Obtains a particular parameter.
     *
     * \param[in] name The name of the parameter.
     * \param[in] type The type of the parameter.
     * \return A pointer to the specified parameter or nullptr if no such parameter exists.
     */
    virtual Parameter * lookupParameter(std::string const & name, ParameterType type);

    /*!
     * Obtains a particular parameter.
     *
     * \param[in] name The name of the parameter.
     * \param[in] type The type of the parameter.
     * \return A pointer to the specified parameter or nullptr if no such parameter exists.
     */
    virtual Parameter const * lookupParameter(std::string const & name, ParameterType type) const;

    /*!
     * Produces a string description of this object.
     *
     * \param[in] os The output stream to which to write the string.
     * \param[in] title The title to include at the beginning of the string.
     * \param[in] prefix What to add to the beginning of each line in the string.
     */
    virtual void dump(std::ostream & os, std::string const & title,
        std::string const & prefix) const;

    /*!
     * Emits events when their conditions are met.
     *
     * \return Whether the operation was successful.
     */
    virtual bool emitEvents();

    /*!
     * Used by subclasses to make one of their fields externally accessible.
     * Once declared, the parameter is available through the methods
     * lookupParameter(...) and getParameterTable(...).
     *
     * \param[in] name The name of the parameter.
     * \param[in] instance The instance variable that underlies the parameter.
     * \param[in] flags The parameter's flags.
     */
    template<class T>
    Parameter * declareParameter(std::string const & name, T * instance,
        unsigned int flags = Parameter::Flag::Default)
    {
        Parameter * entry = ParameterFactory<T>::create(name, flags, instance);
        if (entry != nullptr)
        {
            // CONTROLIT_PR_DEBUG << "Adding parameter '" << name << "' flags: 0x" << std::hex << flags;
            parameterMap.insert(std::make_pair(name, entry));
            return entry;
        }
        CONTROLIT_PR_WARN << "Not adding parameter '" << name << "'. Factory returned NULL value.";
        return entry;
    }

protected:

    /*!
     * A description of the type of this class (defined by sub-classes).
     */
    std::string typeName;

    /*!
     * A description of the instance name of this class (defined by sub-classes).
     */
    std::string instanceName;

    /*!
     * Map of names to parameters
     */
    ParameterMap parameterMap;

    /*!
     * Whether to use the default parameter bindings. Subclasses should use this 
     * to decide whether to instantiate default parameter bindings. It is by default
     * true.
     */
    int useDefaultBindings;
};

/*!
 * This is used by muParser to dynamically obtain and add variables to a ParameterReflection object.
 *
 * \param[in] szName The name of the parameter.
 * \param[in] pUserData A pointer to the ParameterReflection object.
 * \return the value of the parameter.
 */
template<class T>
double* VariableFactory(const char * szName, void * pUserData)
{
    T * pr = static_cast<T *>(pUserData);

    // Lookup the parameter
    Parameter* param = pr->lookupParameter(szName);
    if (param != NULL)
    {
        switch (param->type())
        {
            case PARAMETER_TYPE_INTEGER:
            {
                // Complicated.. cast to non const integer pointer and
                // then reinterpret the interger pointer as a double.. ugg.
                int* pInt = const_cast<int *>(param->getInteger());
                return reinterpret_cast<double*>(pInt);
            }
            case PARAMETER_TYPE_REAL:
            {
                return const_cast<double *>(param->getReal());
            }
            default:
            {
                CONTROLIT_ERROR << "Parameter '" << param->name() << "' is incorrect type: "
                    << Parameter::parameterTypeToString(param->type())
                    << ". Only integer or real supported.";
                throw mu::ParserError("Incorrect parameter type.");
            }
        }
    }

    // We don't have the requested parameter. Add it to the ParameterReflection object.
    double * ptr = new double(0.0);
    param = pr->addParameter(szName, ptr);
    return ptr;
}

} // end namespace controlit

#endif  // __CONTROLIT_CORE_PARAMETER_REFLECTION_HPP__
