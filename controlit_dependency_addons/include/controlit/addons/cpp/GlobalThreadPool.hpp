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

#ifndef __CONTROLIT_ADDONS_CPP_GLOBAL_THREAD_POOL_HPP__
#define __CONTROLIT_ADDONS_CPP_GLOBAL_THREAD_POOL_HPP__

#include <functional>

namespace controlit {
namespace addons {
namespace cpp {

// TODO: Add a init_thread_pool(unsigned int numThreads) API call. Currently
// initialization is hardcoded to 6 threads.

/*! \brief Tries to obtain lock on the global thread pool.
 *  \return True if the lock was obtained.
 *  \note This function is real-time safe.
 */
bool trylock_thread_pool();

/*! \brief Pends until lock is acquired on the global thread pool.
 *  \note This function is not real-time safe.
 */
void lock_thread_pool();

/*! \brief Unlocks the global thread pool.
 *
 *	This must be called after a successfuly trylock_thread_pool() or
 *	lock_thread_pool(). Otherwise, the pool will remain locked and no
 *	jobs will be serviced.
 */
void unlock_thread_pool();

/*! \brief Queues a job and unlocks the thread pool.
 *	\param[in] job The job to perform by the thread pool
 *	\pre Lock must have been obtained through successful trylock_thread_pool() or lock_thread_pool()
 *
 *	This function will both queue the job and unlock, allowing the thread pool
 *	to immediately resume processing.
 */
void queue_and_unlock_thread_pool(std::function<void()> job);

} // namespace cpp
} // namespace addons
} // namespace controlit

#endif