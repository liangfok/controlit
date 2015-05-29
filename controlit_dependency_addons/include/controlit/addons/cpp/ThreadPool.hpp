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

#ifndef __CONTROLIT_ADDONS_CPP_THREAD_POOL_HPP__
#define __CONTROLIT_ADDONS_CPP_THREAD_POOL_HPP__

#include <thread>
#include <mutex>
#include <queue>
#include <functional>
#include <condition_variable>
        

namespace controlit {
namespace addons {
namespace cpp {

/*! Simple threadpool.
 */
class ThreadPool
{
public:
    typedef std::function<void()> Job_t;

    explicit ThreadPool(size_t numThreads);
    ~ThreadPool();

    void lock() {queueMutex_.lock();}
    bool trylock();
    void unlock();

    // USE LOCK OR TRYLOCK FIRST BEFORE CALLING THIS FUNCTION!
    void addJob(Job_t);

    // USE LOCK OR TRYLOCK FIRST BEFORE CALLING THIS FUNCTION!
    void addJobAndUnlock(Job_t);

private:
    void processQueue(size_t id);

    // need to keep track of threads so we can join them
    std::vector< std::thread > workers_;

    // the job queue
    std::queue< Job_t > jobQueue_;

    // synchronization
    std::mutex queueMutex_;
    std::condition_variable cv_;
    bool stop_;
};

} // namespace cpp
} // namespace addons
} // namespace controlit

#endif