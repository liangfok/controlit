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

#include <controlit/addons/cpp/ThreadPool.hpp>

namespace controlit {
namespace addons {
namespace cpp {

// the constructor just launches some amount of workers
ThreadPool::ThreadPool(size_t numThreads) :
    stop_(false)
{
    for(size_t i = 0; i < numThreads; ++i)
    {
        workers_.emplace_back(&ThreadPool::processQueue, this, i);
    }
}

// the destructor joins all threads
ThreadPool::~ThreadPool()
{
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        stop_ = true;
    }

    cv_.notify_all();
    for(size_t i = 0; i < workers_.size(); ++i)
        workers_[i].join();
}

bool ThreadPool::trylock()
{
    return queueMutex_.try_lock();
}

void ThreadPool::unlock()
{
    queueMutex_.unlock();

    // Notify a everyone that there are some jobs waiting
    cv_.notify_all();
}

void ThreadPool::addJob(ThreadPool::Job_t job)
{
    // don't allow adding after stopping the pool
    if (stop_)
        throw std::runtime_error("addJob on stopped ThreadPool");

    jobQueue_.push(job);
}

void ThreadPool::addJobAndUnlock(ThreadPool::Job_t job)
{
    addJob(job);

    // Assumes you got the mutex through lock() or trylock()
    queueMutex_.unlock();

    // Notify a single thread that there is a job waiting
    cv_.notify_one();
}

void ThreadPool::processQueue(size_t id)
{
    while (true)
    {
        // Blocking lock
        std::unique_lock<std::mutex> lock(queueMutex_);

        // Unlock the mutex, wait for the condition to be signaled
        // cv_.wait(lock, [&stop_, &jobQueue_] { return stop_ or (not jobQueue_.empty()); });
        cv_.wait(lock, [this] { return stop_ or (not jobQueue_.empty()); });

        // Re-locked.. check the exit condition
        if (stop_) return;

        // std::cout << "Thread " << id << " doing some work. Queue size: " << jobQueue_.size() << std::endl;

        // Get some work
        ThreadPool::Job_t job(jobQueue_.front());
        jobQueue_.pop();

        // Unlock
        // std::cout << "Thread " << id << " processing job." << std::endl;
        lock.unlock();

        // And process the job
        job();
    }
}

} // namespace cpp
} // namespace addons
} // namespace controlit