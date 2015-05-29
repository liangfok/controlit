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

#ifndef __CONTROLIT_ROBOT_INTERFACE_LIBRARY_SHARED_MEMORY_CONTROL_INTERFACE_HPP__
#define __CONTROLIT_ROBOT_INTERFACE_LIBRARY_SHARED_MEMORY_CONTROL_INTERFACE_HPP__

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <boost/interprocess/exceptions.hpp>

#include <boost/thread/thread_time.hpp>

class SharedMemoryControlInterface
{
public:
  typedef boost::interprocess::allocator<double, boost::interprocess::managed_shared_memory::segment_manager> ShmemDoubleAllocator;
  typedef boost::interprocess::vector<double, ShmemDoubleAllocator> SMDoubleVector;

  typedef boost::interprocess::allocator<char, boost::interprocess::managed_shared_memory::segment_manager> ShmemCharAllocator;
  typedef boost::interprocess::basic_string<char, std::char_traits<char>, ShmemCharAllocator> ShmemString;
  typedef boost::interprocess::allocator<ShmemString, boost::interprocess::managed_shared_memory::segment_manager> ShmemStringAllocator;
  typedef boost::interprocess::vector<ShmemString, ShmemStringAllocator> SMStringVector;

  //each field will be a vector of double, length num_joints
  SharedMemoryControlInterface(std::string interface_name, unsigned long num_joints, std::vector<std::string> field_names, bool force_sm_reset = false)
  {
    std::cerr << "SharedMemoryControlInterface: Constructing Shared Memory.  num_joints = " << num_joints << std::endl;

    try
    {
      m_mutex_name = interface_name + "_mutex";
      m_data_name = interface_name + "_data";

      if(force_sm_reset)
      {
        boost::interprocess::shared_memory_object::remove(m_data_name.c_str());
        boost::interprocess::named_mutex::remove(m_mutex_name.c_str());
      }

      m_mutex = new boost::interprocess::named_mutex(boost::interprocess::open_or_create, m_mutex_name.c_str());

      boost::posix_time::ptime timeout = boost::get_system_time() + boost::posix_time::milliseconds(1000);
      if(m_mutex->timed_lock(timeout))
      {
        m_mutex->unlock();
      }
      else //lock is most likely broken
      {
        std::cerr << "SharedMemoryControlInterface: SHARED MEMORY NOT CLEANED UP PROPERLY! DELETING OLD SPACES!\n"
          << " - m_mutex_name: " << m_mutex_name << "\n"
          << " - m_data_name: " << m_data_name << "\n";

        // delete old shared memory objects
        boost::interprocess::shared_memory_object::remove(m_data_name.c_str());
        boost::interprocess::named_mutex::remove(m_mutex_name.c_str());
        m_mutex = new boost::interprocess::named_mutex(boost::interprocess::open_or_create, m_mutex_name.c_str());
      }

      {
        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex); //force one process to do the whole initialization before the other

        unsigned long data_size = num_joints * sizeof(double) * field_names.size();
        //NOTE: apparently the size of the overhead memory usage is nearly impossible to calculate, so it's recommended that you just make sure to allocate a few pages of memory (4Kb each on linux)
        unsigned long total_num_constructs = field_names.size() + 4;
        unsigned long overhead_size = 4096 * total_num_constructs;
        unsigned long names_size = 4096; //TODO: don't just guess
        unsigned long total_size = overhead_size + data_size + names_size;

        m_segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, m_data_name.c_str(), total_size);
        const ShmemDoubleAllocator alloc_double_inst(m_segment.get_segment_manager());
        const ShmemStringAllocator alloc_string_inst(m_segment.get_segment_manager());

        bool need_to_init = true;
        unsigned long* num_connections = 0;
        try
        {
            num_connections = m_segment.construct<unsigned long>("connection_tokens")(0); //try to make a field
        }
        catch(boost::interprocess::interprocess_exception &ex)
        {
            need_to_init = false; //construct throws an exception when the object already exists
        }

        if(need_to_init)
        {
          std::cerr << "***********************************************************************\n"
                    << "*    SharedMemoryControlInterface: Initializing new shared memory.    *\n"
                    << "***********************************************************************"
                    << std::endl;
          //rtt
          m_rtt_seq_no_tx = m_segment.construct<unsigned char>("rtt_seq_no_tx")(0); //note: the zero at the end initializes the memory to zero
          m_rtt_seq_no_rx = m_segment.construct<unsigned char>("rtt_seq_no_rx")(0);
          m_new_command_flag = m_segment.construct<bool>("new_command_flag")(false);
          m_new_measurement_flag = m_segment.construct<bool>("new_measurement_flag")(false);

          //raw data
          for(unsigned int i = 0; i < field_names.size(); i++)
          {
            SMDoubleVector* vector = m_segment.construct<SMDoubleVector>(field_names[i].c_str())(alloc_double_inst);

            std::cerr << "SharedMemoryControlInterface: Resizing shared memory for " << field_names[i] << " to be of size " << num_joints << std::endl;
            try
            {
              // Note: This operation is dangerous if there is insufficient memory allocated.
              // Thus, the try/catch block surrounding this operation.
              vector->resize(num_joints, 0.0);
            }
            catch(boost::interprocess::interprocess_exception ee)
            {
              std::cerr << "SharedMemoryControlInterface: Exception while resizing SMDoubleVector for field \"" << field_names[i] << "\".  Was there enough shared memory allocated?" << std::endl;
              throw ee;
            }
          }

          m_segment.construct<SMStringVector>("names")(alloc_string_inst);
        }
        else //just find the existing fields
        {
          std::cerr << "***********************************************************************\n"
                    << "* SharedMemoryControlInterface: Connecting to existing shared memory. *\n"
                    << "***********************************************************************"
                    << std::endl;

          num_connections = m_segment.find<unsigned long>("connection_tokens").first;
          m_rtt_seq_no_tx = m_segment.find<unsigned char>("rtt_seq_no_tx").first;
          m_rtt_seq_no_rx = m_segment.find<unsigned char>("rtt_seq_no_rx").first;
          m_new_command_flag = m_segment.find<bool>("new_command_flag").first;
          m_new_measurement_flag = m_segment.find<bool>("new_measurement_flag").first;
        }
        *num_connections = *num_connections + 1;
      }

      m_num_joints = num_joints;
      m_field_names = field_names;
    }
    catch(boost::interprocess::interprocess_exception ee)
    {
      std::cerr << "Exception while initializing shared memory interface!\n"
        << " - error: " << ee.what();
      throw ee;
    }
  }

  ~SharedMemoryControlInterface()
  {
    {
      boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
      unsigned long* num_connections = m_segment.find<unsigned long>("connection_tokens").first;
      if((*num_connections) > 1) //we aren't the last one here
      {
        std::cerr << "Disconnecting from shared memory.\n";
        *num_connections = *num_connections - 1;
        return;
      }
    }
    //if we have reached this point, we are the last one attached to the memory, so delete everything
    std::cerr << "Deleting shared memory.\n";
    boost::interprocess::shared_memory_object::remove(m_data_name.c_str());
    boost::interprocess::named_mutex::remove(m_mutex_name.c_str());
  }

  double getData(const std::string & field, unsigned long joint_idx)
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    return m_segment.find<SMDoubleVector>(field.c_str()).first->at(joint_idx);
  }

  void setData(const std::string & field, unsigned long joint_idx, double value)
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    m_segment.find<SMDoubleVector>(field.c_str()).first->at(joint_idx) = value;
  }

  std::vector<double> getField(const std::string & field)
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    SMDoubleVector* field_data_sm = m_segment.find<SMDoubleVector>(field.c_str()).first;
    std::vector<double> field_data_local;

    field_data_local.resize(field_data_sm->size());
    for(unsigned int i = 0; i < field_data_sm->size(); i++)
    {
      field_data_local.at(i) = field_data_sm->at(i);
    }

    return field_data_local;
  }

  void setField(const std::string & field, const std::vector<double> & field_data_local)
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    SMDoubleVector* field_data_sm = m_segment.find<SMDoubleVector>(field.c_str()).first;

    if(field_data_local.size() != field_data_sm->size())
    {
      ROS_ERROR("SMCI: setField: Tried to set field %s of size %d with data of size %d!", field.c_str(), (int) field_data_sm->size(), (int) field_data_local.size());
    }

    for(unsigned int i = 0; i < field_data_sm->size(); i++)
    {
      field_data_sm->at(i) = field_data_local.at(i);
    }
  }

  std::vector<std::string> getJointNames()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    std::vector<std::string> names_local;
    SMStringVector* names_sm = m_segment.find<SMStringVector>("names").first;
    std::cerr << "\ngetJointNames: Getting joint names." << std::endl;

    if(names_sm == 0) //bad ptr
    {
      return names_local;
    }

    for(unsigned int i = 0; i < names_sm->size(); i++)
    {
      std::string name = std::string(names_sm->at(i).begin(), names_sm->at(i).end());
      names_local.push_back(name);

      std::cerr << "getJointNames: " << i << ": " << name << std::endl;
    }

    std::cerr << "getJointNames: Done.\n" << std::endl;
    return names_local;
  }

  void setJointNames(const std::vector<std::string> & names_local)
  {
    // Print joint names prior to being saved in shared memory
    // std::stringstream ss1;
    // for (size_t ii = 0; ii < names_local.size(); ii++)
    // {
    //   ss1 << "  - " << ii << ": " << names_local[ii];
    //   if (ii < names_local.size() - 1) ss1 << "\n";
    // }
    // std::cerr << "SMCI: setJointNames: Method Called! names_local:\n" << ss1.str() << std::endl;

    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    const ShmemCharAllocator alloc_char_inst(m_segment.get_segment_manager());
    SMStringVector* names_sm = m_segment.find<SMStringVector>("names").first;

    names_sm->clear();

    if (names_local.size() != m_num_joints)
    {
      ROS_ERROR("SMCI: setJointNames: Tried to set joint names of size %d with data of size %d!", (int) m_num_joints, (int) names_local.size());
    }


    for(unsigned int i = 0; i < names_local.size(); i++)
    {
      ShmemString name(alloc_char_inst);
      name = names_local.at(i).c_str();
      names_sm->push_back(name);
    }


    // Verify that the joint names were set correctly in shared memory
    bool problemDetected = false;
    for (size_t ii = 0; ii < names_sm->size() && !problemDetected; ii++)
    {
      if (names_local[ii].compare(std::string(names_sm->at(ii).begin(), names_sm->at(ii).end())) != 0)
        problemDetected = true;
    }

    if (problemDetected)
    {
      std::stringstream ss;
      ss << "SMCI: setJointNames: WARNING: Problems detected when setting joint names:\n"
         << "  - Original joint names:\n";

      for (size_t ii = 0; ii < names_local.size(); ii++)
      {
        ss << "    - " << ii << ": " << names_local[ii];
        if (ii < names_local.size() - 1) ss << "\n";
      }

      ss << "  - Joint names in shared memory:\n";

      for (size_t ii = 0; ii < names_sm->size(); ii++)
      {
        ss << "    - " << ii << ": " << std::string(names_sm->at(ii).begin(), names_sm->at(ii).end());
        if (ii < names_sm->size() - 1) ss << "\n";
      }

      std::cerr << ss.str() << std::endl;
    }



    // Print joint names after to being saved in shared memory
    // std::stringstream ss2;
    // for (size_t ii = 0; ii < names_sm->size(); ii++)
    // {
    //   ss2 << "  - " << ii << ": " << std::string(names_sm->at(ii).begin(), names_sm->at(ii).end());
    //   if (ii < names_sm->size() - 1) ss2 << "\n";
    // }
    // std::cerr << "SMCI: setJointNames: Done settings joint names:\n" << ss2.str() << std::endl;
  }

  void setTxSequenceNumber(unsigned char value)
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    *m_rtt_seq_no_tx = value;
  }

  unsigned char getTxSequenceNumber()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    return *m_rtt_seq_no_tx;
  }

  void setRxSequenceNumber(unsigned char value)
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    *m_rtt_seq_no_rx = value;
  }

  unsigned char getRxSequenceNumber()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    return *m_rtt_seq_no_rx;
  }

  bool hasConnections()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    unsigned long* num_connections = m_segment.find<unsigned long>("connection_tokens").first;
    return (*num_connections) > 1;
  }

  bool hasNewCommand()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    return *m_new_command_flag;
  }

  void signalCommandAvailable()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    *m_new_command_flag = true;
  }

  void signalCommandProcessed()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    *m_new_command_flag = false;
  }

  bool hasNewMeasurement()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    return *m_new_measurement_flag;
  }

  void signalMeasurementAvailable()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    *m_new_measurement_flag = true;
  }

  void signalMeasurementProcessed()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    *m_new_measurement_flag = false;
  }

  void printFields()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    for(unsigned int i = 0; i < m_field_names.size(); i++)
    {
      std::cerr << m_field_names[i] << ":\t";
      SMDoubleVector* field_data_sm = m_segment.find<SMDoubleVector>(m_field_names[i].c_str()).first;
      for(unsigned int j = 0; j < field_data_sm->size(); j++)
      {
        std::cerr << field_data_sm->at(j);
        if(j == (field_data_sm->size() - 1))
        {
          std::cerr << std::endl;
        }
        else
        {
          std::cerr << ", ";
        }
      }
    }
  }

private:
  boost::interprocess::named_mutex* m_mutex;
  boost::interprocess::managed_shared_memory m_segment;
  std::vector<std::string> m_field_names;
  unsigned long m_num_joints;
  std::string m_mutex_name;
  std::string m_data_name;
  unsigned char* m_rtt_seq_no_tx;
  unsigned char* m_rtt_seq_no_rx;
  bool* m_new_command_flag;
  bool* m_new_measurement_flag;
};

#endif //__CONTROLIT_ROBOT_INTERFACE_LIBRARY_SHARED_MEMORY_CONTROL_INTERFACE_HPP__
