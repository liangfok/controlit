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

#ifndef __RX_THREAD_HPP__
#define __RX_THREAD_HPP__

#include <unistd.h>
#include <pthread.h>
#include <string.h>

namespace controlit_udp {

	class RxThread
	{
	public:
		RxThread(){}
		virtual ~RxThread(){}

	protected:
		bool StartRxThread()
		{
			return (pthread_create(&t_, NULL, RxThreadEntryFunc, this) == 0);
		}

		void WaitForRxThreadExit()
		{
			(void) pthread_join(t_, NULL);
		}

		//Implement this method in child classes with loop
		virtual void RxThreadEnterLoop() = 0;

	private:
		static void * RxThreadEntryFunc(void * This)
		{
			((RxThread *)This)->RxThreadEnterLoop();
			return NULL;
		}

		pthread_t t_;
	};

} //namespace controlit_udp
#endif // __RX_THREAD_HPP__