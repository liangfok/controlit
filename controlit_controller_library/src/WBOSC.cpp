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

#include <controlit/controller_library/WBOSC.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/addons/eigen/PseudoInverse.hpp>
#include <controlit/Task.hpp>

#include <math.h>
#include <sys/time.h>

#include <iomanip>  // For printing full floating point precision,
                    // see std::setprecision below

// For publishing the equivalent embedded damping gains
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sstream>
#include <controlit/utility/string_utility.hpp>

namespace controlit {
namespace controller_library {

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG_RT << ss;

// #define PRINT_DEBUG_STATEMENT_RT_ALWAYS(ss) CONTROLIT_DEBUG_RT << ss;
#define PRINT_DEBUG_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

WBOSC::WBOSC()
    : Controller("Unnamed"),
      controlitParameters(nullptr)
{
}

WBOSC::WBOSC(std::string const& name)
    : Controller(name),
      controlitParameters(nullptr)
{
}

bool WBOSC::init(ros::NodeHandle & nh, ControlModel & model,
    controlit::utility::ControlItParameters * controlitParameters,
    std::shared_ptr<Timer> timer)
{
    // Ensure this WBOSC is initialized once
    assert(!initialized);

    // Save the pointer to the object holding WBC's parameters
    this->controlitParameters = controlitParameters;

    this->timer = timer;

    // Initialize the gravity compensation vector publisher
    gravityCompensationPublisher.init(nh, model.getActuatedJointNamesVector());

    if (reinit(model))
    {
        initialized = true;
        return true;
    }
    else
        return false;
}

bool WBOSC::reinit(ControlModel & model)
{
    // Get the number of actuable DOFs
    int numDOFs = model.getNActuableDOFs();

    // Reset the gravityComp vector
    if (gravityComp.size() != numDOFs)
    {
        gravityComp.resize(numDOFs);
        gravityComp.setZero(numDOFs);
    }

    // Reset the identity matrix with size numDOFs x numDOFs
    identityActuableDOFs.setIdentity(numDOFs, numDOFs);

    // Initialize the fcomp and pstar vectors
    pstar.setZero(numDOFs); // This is over-allocating. fcomp at each level is # of rows in Jacobien.
    fcomp.setZero(numDOFs); // This is over-allocating. pstar at each level is # of rows in Jacobian.

    // Save the actuated joint names.  This is used by the getEquivalentDampingGainsHandler service.
    // actuatedJointNames = & model.getActuatedJointNamesVector();

    return true;
}

bool WBOSC::computeCommand(ControlModel & model, CompoundTask & compoundTask, Command & command)
{
    // CONTROLIT_DEBUG_RT << "Method called! \n"
    //              " - model.getQ() = " << model.getQ().transpose();

    // #define TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND 1

    #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND
    timer->start();  // reset the timer
    #endif

    // Get the number of actuable DOFs
    int numDOFs = model.getNActuableDOFs();

    // Ensure output variables are property allocated
    assert(command.getEffortCmd().size() == numDOFs);
    assert(command.getPositionCmd().size() == numDOFs);
    assert(command.getVelocityCmd().size() == numDOFs);
    assert(command.getEffectiveKp().size() == numDOFs);
    assert(command.getEffectiveKd().size() == numDOFs);

    // Get references to various useful matricies and vector.
    const Matrix & Ai = model.getAinv();
    const Matrix & UNcAiNorm = model.constraints().getUNcAiNorm();
    const Matrix & UNcBar = model.constraints().getUNcBar();
    // const Matrix & Nc = model.constraints().getNc();
    // const Matrix & UNc = model.constraints().getUNc();
    // const Vector & grav = model.getGrav();

    // Initialize Nhp, the null space of higher priority tasks, to be identity.
    Nhp = identityActuableDOFs;

    // CONTROLIT_DEBUG_RT << "Input variables:\n"
    //      " - ControlModel name = " << model.getName() << "\n"
    //      " - numDOFs = " << numDOFs << "\n"
    //      " - Ai = \n" << Ai << "\n"
    //      " - grav = " << grav.transpose();

    CompoundTask::TaskCommands taskCommands;
    CompoundTask::TaskJacobians taskJacobians;
    CompoundTask::TaskTypes taskTypes;

    #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND
    timeBookkeeping = timer->getTime();
    #endif

    if (!compoundTask.getJacobianAndCommand(model, taskJacobians, taskCommands, taskTypes))
        return false;

    #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND
    timeGetJacobianAndCommand = timer->getTime() - timeBookkeeping;
    #endif

    bool hasInternalForceTask = compoundTask.hasInternalForceTask();
    size_t internalForceTaskPriority = compoundTask.getInternalForceTaskPriority();
    int numPrevTasks = 0;

    gravityComp.setZero(numDOFs);

    #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND
    std::vector<double> latencyComputeTaskCommand;
    #endif

    // For each task priority level
    for(size_t priority = 0; priority < taskCommands.size(); priority++)
    {
        // CONTROLIT_DEBUG_RT << "Processing priority: " << priority;

        #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND
        timer->start(); // reset the timer
        #endif

        // If the priority level is not empty and does not belong to the internal force task
        if(taskCommands[priority].size() > 0 &&
          (!hasInternalForceTask || (hasInternalForceTask && priority != internalForceTaskPriority)))
        {
            // CONTROLIT_INFO_RT << "Computing Jstar:\n"
            //   << " - Priority: " << priority << "\n"
            //   << " - size of taskJacobians[" << priority << "]: (" << taskJacobians[priority].rows() << "x" << taskJacobians[priority].cols() << ")\n"
            //   << " - size of taskCommands[" << priority << "]: " << taskCommands[priority].size() << "\n"
            //   << " - size of UNcBar: (" << UNcBar.rows() << "x" << UNcBar.cols() << ")\n"
            //   << " - size of Nhp: (" << Nhp.rows() << "x" << Nhp.cols() << ")";

            // Jstar tells you the feasibility of the task.  In other words it expresses the task space.
            // For example, if your legs are straight, you cannot move anymore.
            Matrix Jstar = taskJacobians[priority] * UNcBar * Nhp; //Nhp is identity for top level task.  It is the nullspace of all higher priority tasks

            // CONTROLIT_DEBUG_RT << "Done computing Jstar";

            // CONTROLIT_DEBUG_RT << "Computing inverseLstar:\n"
            //   << " - Priority: " << priority << "\n"
            //   << " - size of Jstar: (" << Jstar.rows() << "x" << Jstar.cols() << ")\n"
            //   << " - size of UNcAiNorm: (" << UNcAiNorm.rows() << "x" << UNcAiNorm.cols() << ")";

            // TODO: Remove this dynamic allocation
            Matrix inverseLstar = Jstar * UNcAiNorm * Jstar.transpose();

            // Lstar tells you the ability to do something dynamic.
            // For example, if you spin your arms fast enough, maybe you can lift off the ground.
            // TODO: Remove this dynamic allocation
            Lstar.resize(inverseLstar.cols(), inverseLstar.rows());

            // std::cout<<"Lstar["<<priority<<"] = \n"<<Lstar<<std::endl;

            // CONTROLIT_DEBUG_RT << "Done computing inverseLstar";

            // CONTROLIT_DEBUG_RT
            //   // << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
            //   << "Details of matrix to be inverted:\n"
            //   << " - Priority: " << priority << "\n"
            //   << " - Matrix being inverted:\n" << inverseLstar << "\n"
            //   << " - taskJacobians:\n" << taskJacobians[priority] << "\n"
            //   << " - UNcBar:\n" << UNcBar << "\n"
            //   << " - Nhp:\n" << Nhp;

            // std::stringstream msgBuff;
            // for (int ii = 0; ii < inverseLstar.rows(); ii++)
            // {
            //   for (int jj = 0; jj < inverseLstar.cols(); jj++)
            //   {
            //     if (jj != 0)
            //       msgBuff << ", ";
            //     msgBuff << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1) << inverseLstar(ii, jj);
            //   }
            //   msgBuff << ",\n";
            // }
            // CONTROLIT_DEBUG_RT << "Here's code for initializing the matrix.  Size = " << inverseLstar.rows() << "x" << inverseLstar.cols() << ":\n" << msgBuff.str();

            // ros::Time startMethodCall = ros::Time::now();

            // CONTROLIT_INFO << "Computing pseudoInverse";
            controlit::addons::eigen::pseudo_inverse(inverseLstar, Lstar); //, compoundTask.getSigmaThreshold());

            // CONTROLIT_DEBUG_RT << "Done computing Lstar";

            // for(size_t ii = 0; ii < Lstar.rows(); ii++)
            //   for(size_t jj = 0; jj < Lstar.rows(); jj++)
            //     if(ii != jj)
            //     {
            //       Lstar(ii,jj) *= 0.3;
            //       Lstar(jj,ii) *= 0.3;
            //     }

            // CONTROLIT_DEBUG_RT << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
            //   << __func__ << ": Results of controlit::addons::eigen::pseudo_inverse:\n" << Lstar;

            // double latency = (ros::Time::now() - startMethodCall).toSec();

            // CONTROLIT_DEBUG_RT << "Latency of controlit::addons::eigen::pseudo_inverse method call:\n"
            //      " - priority level: " << priority << "\n"
            //      " - size of input matrix: " << temp.rows() << "x" << temp.cols() << "\n"
            //      // " - max singular value: " << singularValues.maxCoeff() << "\n"
            //      // " - min singular value: " << singularValues.minCoeff() << "\n"
            //      // " - condition of input matrix: " << (singularValues.minCoeff() > 0 ? (singularValues.maxCoeff() / singularValues.minCoeff()) : -1) << "\n"
            //      " - latency: " << latency * 1e3 << "\n"
            //      " - numRounds: " << numRounds;

            // This is debug code that sets Lstar = identity, which effectively
            // disables WBC.
            // Lstar.setIdentity(model.getNActuableDOFs(), model.getNActuableDOFs());

            // Compute pstar
            // Dimensions:
            //  - pstar = # task DOFs
            //  - Lstar = # actuable DOFs x # actuable DOFs
            //  - Jstar = # actuable DOFs x # actuable DOFs
            //  - UNc = # actuable DOFs x # DOFs
            //  - Ai = # DOFs x # DOFs
            //  - Nc.transpose = # DOFs x # DOFs
            //  - grav = # DOFs



            pstar.setZero(Lstar.rows());  // disable task-specific gravity comp
            // pstar = Lstar * Jstar * UNc * Ai * Nc.transpose()* grav;
            // pstar.setZero();
            // std::cout<<"pstar = "<<pstar.transpose()<<"std::endl";
            // gravityComp.noalias() += pstar;

            // CONTROLIT_DEBUG_RT << "Done setting pstar";

            // Print this to determine the "effective" gains.
            // The "effective" gains are the actual gains * the corresponding value in the matrix's diagnal.
            // CONTROLIT_DEBUG_RT << "Lstar:\n" << Lstar;

            if (numPrevTasks == 0)
            {
                if(taskTypes[priority] == CommandType::ACCELERATION)
                    command.getEffortCmd() = Jstar.transpose() * (Lstar * taskCommands[priority] + pstar);
                else //CommandType::FORCE
                    command.getEffortCmd() = Jstar.transpose() * (taskCommands[priority] + pstar);

                // CONTROLIT_DEBUG_RT << "Controller State:\n"
                //      " - gamma = " << gamma.transpose() << "\n"
                //      " - Q = " << model.getQ().transpose() << "\n"
                //      " - Qd = " << model.getQd().transpose() << "\n"
                //      " - Jstar = \n" << Jstar << "\n"
                //      " - Lstar = \n" << Lstar << "\n"
                //      " - pstar = " << pstar.transpose() << "\n"
                //      " - grav = " << grav.transpose() << "\n"
                //      // " - Lstar = \n" << Lstar << "\n"
                //      // " - UNc = \n" << UNc << "\n"
                //      // " - Ai = \n" << Ai << "\n"
                //      // " - Nc.transpose() = \n" << Nc.transpose()
                //      " - Jstar.transpose() * Lstar * taskCommands[" << priority << "] = "
                //        << (Jstar.transpose() * Lstar * taskCommands[priority]).transpose()
                //      ;
                // return false;
            }
            else
            {
                fcomp.setZero(Lstar.rows());
                fcomp = Lstar * Jstar * UNcAiNorm * command.getEffortCmd();
                if(taskTypes[priority] == CommandType::ACCELERATION)
                    command.getEffortCmd().noalias() += Jstar.transpose() * (Lstar * taskCommands[priority] + pstar - fcomp);
                else //CommandType::FORCE
                    command.getEffortCmd().noalias() += Jstar.transpose() * (taskCommands[priority] + pstar - fcomp);
            }

            if (!containerUtility.checkMagnitude(command.getEffortCmd(), INFINITY_THRESHOLD))
            {
                CONTROLIT_ERROR_RT << "Invalid effortCmd after accounting for priority " << priority << " tasks!\n"
                    // << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
                    << " - effortCmd = " << command.getEffortCmd().transpose() << "\n"
                       " - Q = " << model.getQ().transpose() << "\n"
                       " - Qd = " << model.getQd().transpose() << "\n"
                       " - Ainv = \n" << Ai << "\n"
                       " - A = \n" << model.getA() << "\n"
                       " - Jstar = \n" << Jstar << "\n"
                       " - UNcAiNorm = \n" << UNcAiNorm << "\n"
                       " - inverseLstar = \n" << inverseLstar << "\n"
                       " - Lstar = \n" << Lstar << "\n"
                       " - taskCommands[" << priority << "] = " << taskCommands[priority].transpose() << "\n"
                       " - pstar = " << pstar.transpose() << "\n"
                       " - fcomp = " << fcomp.transpose();

                return false;
            }

            if (priority < taskCommands.size() - 1) // Avoid last calculation
            {
                Nhp = (identityActuableDOFs - UNcAiNorm * Jstar.transpose() * Lstar * Jstar) * Nhp; //check order of projection
            }

            numPrevTasks++;
        }

        #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND
        latencyComputeTaskCommand.push_back(timer->getTime());
        #endif
    }

    #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND
    timer->start(); // reset the timer
    #endif

    // Calculate and add joint space gravity compensation to the command.
    // Then publish the gravity compensation vector for debugging and monitoring purposes.
    gravityComp = UNcBar.transpose() * model.getGrav();
    command.getEffortCmd().noalias() += gravityComp;
    gravityCompensationPublisher.publish(gravityComp, model.getActuableQ(), model.getActuableQd());

    #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND
    timeAddGravityComp = timer->getTime();
    #endif

    // CONTROLIT_INFO << "\n"
    //   << "gravityComp = " << gravityComp.transpose() << "\n"
    //   << "UNcBar: " << UNcBar << "\n"
    //   << "Ainv: " << Ai;
    // std::cout<<"using normal gravity comp"<<std::endl;

    //  CONTROLIT_DEBUG_RT << "The Command is: " << command.transpose() << "\n"
    //       " - taskCommands[0] = " << taskCommands[0].transpose() << "\n"
    //       " - taskJacobians[0] =\n" << taskJacobians[0] << "\n"
    //       " - UNcBar =\n" << UNcBar;

    if (!containerUtility.checkMagnitude(command.getEffortCmd(), INFINITY_THRESHOLD))
    {
        CONTROLIT_ERROR_RT << "Invalid effortCmd after accounting for regular tasks!\n"
            << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
            << " - effortCmd = " << command.getEffortCmd().transpose() << "\n"
               " - Q = " << model.getQ().transpose() << "\n"
               " - Qd = " << model.getQd().transpose();
        return false;
    }

    #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND
    timeCheckMagnitude = timer->getTime() - timeAddGravityComp;
    #endif

    // Determine whether the force task is enabled
    bool forceTaskEnabled = false;
    if (hasInternalForceTask)
        forceTaskEnabled = compoundTask.isTaskEnabled(internalForceTaskPriority, 0);


    #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND
    timeCheckForceTaskEnabled = timer->getTime() - timeCheckMagnitude;
    #endif

    // CONTROLIT_DEBUG_RT << "forceTaskEnabled = " << forceTaskEnabled;

    // All of the task commands are calculated.  Now add VLM commands if necessary.
    if (model.virtualLinkageModel().exists() && forceTaskEnabled)
    {
        // Get references to various useful matricies and vectors.
        const Matrix & Jsbar = model.virtualLinkageModel().getJacobianBar();
        const Matrix & Wint = model.virtualLinkageModel().getWint();
        const Matrix & Lstar = model.virtualLinkageModel().getLstar();
        const Matrix & U = model.virtualLinkageModel().getU();

        Jlstarbar = Lstar * U * Jsbar * Wint.transpose();

        // TODO: Move this into an init method
        Jlstar.resize(Jlstarbar.cols(), Jlstarbar.rows());

        // CONTROLIT_INFO << "Computing pseudoInverse";
        controlit::addons::eigen::pseudo_inverse(Jlstarbar, Jlstar); //, compoundTask.getSigmaThreshold());

        //Matrix Id7(7,7); Id7.setIdentity();
        //std::cout<<"Jlstar * Jlstarbar = \n"<<(Jlstar * Jlstarbar)<<std::endl;

        pl = Wint * Jsbar.transpose() * model.getGrav();

        Fint = Wint * Jsbar.transpose() * U.transpose() * command.getEffortCmd(); // getEffortCmd() is the sum of all operation and joint space tasks.

        if (!containerUtility.checkMagnitude(Fint, INFINITY_THRESHOLD))
        {
            CONTROLIT_ERROR_RT << "Invalid Fint!\n"
                << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
                << " - effortCmd = " << command.getEffortCmd().transpose() << "\n"
                   " - U =\n" << U << "\n"
                   " - Jsbar = " << Jsbar << "\n"
                   " - Wint = " << Wint;
            return false;
        }

        FintRef = taskCommands[internalForceTaskPriority];

        // CONTROLIT_DEBUG_RT << "About to add VLM command:\n"
        //        " - Size of Lstar.transpose(): (" << Lstar.transpose().rows() << "x" << Lstar.transpose().cols() << ")\n"
        //        " - Size of Jlstar.transpose(): (" << Jlstar.transpose().rows() << "x" << Jlstar.transpose().cols() << ")\n"
        //        " - Size of FintRef: " << FintRef.size() << "\n"
        //        " - Size of Fint: " << Fint.size() << "\n"
        //        " - Size of pl: " << pl.size();


        // Vector intCommand = Lstar.transpose() * Jlstar.transpose() * (FintRef - Fint + pl);
        // Vector check = UNc.transpose() * intCommand;
        //std::cout<<"norm of UNc.transpose * Lstar.transpose * Jlstar.transpose = \n"<<UNc.transpose() * Lstar.transpose() * Jlstar.transpose()<<std::endl;
        // std::cout<<"norm of UNc.transpose * intCommand = "<<check.norm()<<std::endl;
        //std::cout<<"norm of UNc.transpose * Lstar.transpose = "<<(UNc.transpose() * Lstar.transpose()).norm()<<std::endl;

        command.getEffortCmd().noalias() += Lstar.transpose() * Jlstar.transpose() * (FintRef - Fint + pl); //Check sign of pl.

        if (!containerUtility.checkMagnitude(command.getEffortCmd(), INFINITY_THRESHOLD))
        {
            CONTROLIT_ERROR_RT << "Invalid effortCmd after accounting for internal force task!\n"
                // << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
                << " - effortCmd = " << command.getEffortCmd().transpose() << "\n"
                   " - Fint = " << Fint.transpose() << "\n"
                   " - FintRef = " << FintRef.transpose() << "\n"
                   " - pl = " << pl.transpose() << "\n"
                   " - Q = " << model.getQ().transpose() << "\n"
                   " - Qd = " << model.getQd().transpose() << "\n"
                   " - Lstar =\n" << Lstar << "\n"
                   " - Jlstar =\n" << Jlstar << "\n";
            return false;
        }
    }

    #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND
    timeAddVLM = timer->getTime() - timeCheckForceTaskEnabled;
    #endif

    if (!containerUtility.checkMagnitude(command.getEffortCmd(), INFINITY_THRESHOLD))
    {
        CONTROLIT_ERROR_RT << "Invalid effortCmd at end of method!\n"
            << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
            << " - effortCmd = " << command.getEffortCmd().transpose() << "\n"
               " - Q = " << model.getQ().transpose() << "\n"
               " - Qd = " << model.getQd().transpose();
        // return false;
    }

    #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND
    timeCheckMagnitude2 = timer->getTime() - timeAddVLM;
    #endif

    #ifdef TIME_TORQUE_CONTROLLER_COMPUTE_COMMAND

    double total = timeBookkeeping + timeGetJacobianAndCommand + timeAddGravityComp +
        timeCheckMagnitude + timeCheckForceTaskEnabled + timeAddVLM +
        timeCheckMagnitude2;

    std::stringstream msgBuff;
    for (size_t ii = 0; ii < latencyComputeTaskCommand.size(); ii++)
    {
        msgBuff << "\n   - addTaskCommandPriority " << ii << ": " << latencyComputeTaskCommand[ii] * 1000;
        total += latencyComputeTaskCommand[ii];
    }

    PRINT_DEBUG_STATEMENT_RT_ALWAYS("WBOSC Compute Command Latency Results in ms (total: " << total * 1000 << "):\n"
        "  - timeBookkeeping: " << timeBookkeeping * 1e3 << "\n"
        "  - timeGetJacobianAndCommand: " << timeGetJacobianAndCommand * 1e3 << "\n"
        "  - prioritized nullspace projection:" << msgBuff.str() << "\n"
        "  - timeAddGravityComp: " << timeAddGravityComp * 1e3 << "\n"
        "  - timeCheckMagnitude: " << timeCheckMagnitude * 1e3 << "\n"
        "  - timeCheckForceTaskEnabled: " << timeCheckForceTaskEnabled * 1e3 << "\n"
        "  - timeAddVLM: " << timeAddVLM * 1e3 << "\n"
        "  - timeCheckMagnitude2: " << timeCheckMagnitude2 * 1e3);
    #endif

    // CONTROLIT_DEBUG_RT << "Done method call:\n"
    //                 " - command = " << command.transpose();

    return true;
}

} // namespace controller_library
} // namespace controlit
