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

#ifndef __CONTROLIT_CORE_REFLECTION_REGISTRY_HPP__
#define __CONTROLIT_CORE_REFLECTION_REGISTRY_HPP__

#include <map>
#include <vector>

#include <controlit/ParameterReflection.hpp>

namespace controlit {

/*!
 * Maintains a set of ParameterReflection objects.
 */
class ReflectionRegistry : public ParameterReflection
{
public:

    /*!
     * Defines the delimeter that separates the name of the object from the name of the parameter.
     */
    static const char ParameterNameDelimiter = '.';

    /*!
     * Defines the type of the map for storing the ParameterReflection objects.
     */
    typedef std::map<std::string, std::shared_ptr<ParameterReflection> > ParameterCollectionMap_t;

    /*!
     * The constructor.
     *
     * \param[in] type_name The name of this reflection registry type.
     * It is usually the name of the sublclass.
     *
     * \param[in] instance_name The name of this particular instance.
     * This can be any arbitrary name.
     */
    ReflectionRegistry(std::string const& type_name,
                       std::string const& instance_name);

    /*!
     * Adds a ParameterReflection object to this registry.
     *
     * \param[in] instance The parameter reflection object to add to this registry.
     * \return Whether the operation was successful.
     */
    bool addParameterCollection(std::shared_ptr<ParameterReflection> instance);

    /*!
     * Removes all ParameterReflection objects from this registry.
     */
    virtual void clearParameterCollection();

    /*!
     * Obtains a ParameterReflection object that is stored in this registry.
     *
     * \param[in] name The name of the ParameterReflection object to obtain.
     * \return The requested ParameterReflection object.  If no such ParameterReflection exists
     * this this registry, NULL is returned.
     */
    std::weak_ptr<ParameterReflection> getParameterCollection(std::string const& name);

    /*!
     * Obtains all of the ParameterReflection objects stored in this registry.
     */
    ParameterCollectionMap_t const& getParameterCollections() const {return parameterCollections;}

    /*!
     * Adds an event.  An event is triggered by an expression that evaluates to true.
     *
     * \param[in] name The name of the event.
     * \param[in] expr The expression that is used to trigger the event.
     * \return Whether the operation was successful.
     */
    virtual bool addEvent(std::string const& name, std::string const& expr);

    /*!
     * Iterates through the ParameterReflection objects in this registry and calls
     * emitEvents() on them.  It then calls emitEvents() on itself.
     * This should be called each time the robot's state changes.
     */
    virtual bool emitEvents();

    /*!
     * Determines whether a parameter of the specified name exists.
     *
     * \param name The name of the parameter.
     * \return true if the parameter exists.
     */
    virtual bool hasParameter(std::string const& name);

    /*!
     * Searches for and returns a particular parameter.
     * It searches through all of the ParameterReflection objects
     * within this registry and the parameters belonging to this
     * ReflectionRegistry object.
     *
     * \return A pointer to the requested parameter, or nullptr if no such
     * parameter exists.
     */
    virtual Parameter* lookupParameter(std::string const& name);

    /*!
     * Searches for and returns a particular parameter.
     * It searches through all of the ParameterReflection objects
     * within this registry and the parameters belonging to this
     * ReflectionRegistry object.
     *
     * \return A pointer to the requested parameter, or nullptr if no such
     * parameter exists.
     */
    virtual Parameter const* lookupParameter(std::string const& name) const;

    /*!
     * Searches for and returns a parameter with a particular name and type.
     * It searches through all of the ParameterReflection objects
     * within this registry and the parameters belonging to this
     * ReflectionRegistry object.
     *
     * \return A pointer to the requested parameter, or nullptr if no such
     * parameter exists.
    */
    virtual Parameter* lookupParameter(std::string const& name, ParameterType type);

    /*!
     * Searches for and returns a parameter with a particular name and type.
     * It searches through all of the ParameterReflection objects
     * within this registry and the parameters belonging to this
     * ReflectionRegistry object.
     *
     * \return A pointer to the requested parameter, or nullptr if no such
     * parameter exists.
    */
    virtual Parameter const* lookupParameter(std::string const& name, ParameterType type) const;

    /*!
     * Gets a string representation of the parameters stored in this reflection registry.
     *
     * \param[out] keys A reference to where the names of the parameters should be stored.
     * \param[out] values A reference to where the values of the parameters should be stored.
     */
    virtual bool getParameters(std::vector<std::string> & keys, std::vector<std::string> & values) const;

    /*!
     * Produces a string representation of the parameters stored in this reflection registry.
     *
     * \param[in] os The output stream to which to write the string.
     * \param[in] title The title of the string.
     * \param[in] prefix A prefix to add to the beginning of each line in the string.
     */
    virtual void dump(std::ostream& os, std::string const& title,
        std::string const& prefix) const;

private:
    // define the map iterators
    typedef ParameterCollectionMap_t::iterator MapIterator_t;
    typedef ParameterCollectionMap_t::const_iterator ConstMapIterator_t;

    /*!
     * A helper method for the lookupParameter and hasParameter methods.
     *
     * \param qualifiedParameterName The name of the parameter.
     * \return A pointer to the parameter with the given name or NULL if
     * such a parameter does not exist.
     */
    Parameter* getParameter(std::string const& qualifiedParameterName);

    /*!
     * This is the actual data structure where the ParameterReflection objects
     * are stored.
     */
    ParameterCollectionMap_t parameterCollections;
};

} // end namespace controlit

#endif // __CONTROLIT_CORE_REFLECTION_REGISTRY_HPP__
