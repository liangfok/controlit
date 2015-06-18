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

#include <controlit/task_library/PDController.hpp>
#include <controlit/Task.hpp>
#include <list>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace impl {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << " (" << controlit::task_library::SaturationPolicy::SaturationPolicyToString(SaturationPolicy) << "): " << ss;

#define PRINT_DEBUG_STATEMENT_NOPOLICY(ss)
// #define PRINT_DEBUG_STATEMENT_NOPOLICY(ss) CONTROLIT_DEBUG_RT << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG_RT << " (" << controlit::task_library::SaturationPolicy::SaturationPolicyToString(SaturationPolicy) << "): " << ss;

#define PRINT_DEBUG_STATEMENT_NOPOLICY_RT(ss)
// #define PRINT_DEBUG_STATEMENT_NOPOLICY_RT(ss) CONTROLIT_DEBUG_RT << ss;

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

namespace interface = controlit::task_library;

/****************************************************************************
 * Default policy (OFF)
 ****************************************************************************/
template<int SaturationPolicy>
struct PDControllerTraits
{
    typedef Vector  Gain;
    typedef Vector  Parameter;
};

template<int SaturationPolicy>
struct PDController : public interface::PDController
{
    typedef typename PDControllerTraits<SaturationPolicy>::Gain     Gain;
    typedef typename PDControllerTraits<SaturationPolicy>::Parameter  Parameter;

    virtual void declareParameters(controlit::ParameterReflection* pr)
    {
        PRINT_DEBUG_STATEMENT("Method Called!")
    
        pr->declareParameter("kp", &kp);
        pr->declareParameter("kd", &kd);
        pr->declareParameter("maxVelocity", &maxVelocity);
        pr->declareParameter("maxAcceleration", &maxAcceleration);
        paramErrorNorm = pr->declareParameter("errorNorm", &e_norm);
        paramErrorDotNorm = pr->declareParameter("errorDotNorm", &e_dot_norm);
        paramError = pr->declareParameter("error", &e);
        paramErrorDot = pr->declareParameter("errorDot", &e_dot);
        paramCommand = pr->declareParameter("PDCommand", &PDCommand);
    
        // Integral related terms
        integralOn = 0;  // default value
        windUpCount = 0;
    
        pr->declareParameter("ki", & ki);
        pr->declareParameter("integralSaturation", & integralSaturation);
        paramIntegralTerm = pr->declareParameter("integralTerm", & integralTerm);
        pr->declareParameter("integralOn", & integralOn);
        pr->declareParameter("integralPeriod", & integralPeriod);
        pr->declareParameter("dt", & dt);
    }
  
    virtual bool resize(int dimension, bool initDefault)
    {
        PRINT_DEBUG_STATEMENT("Method Called!")
    
        if (kp.size() != dimension)
        {
            if (initDefault)
                kp.setZero(dimension);
            else
            {
                CONTROLIT_ERROR << "Size of kp incorrect.  Expected " << dimension << ", got " << kp.size();
                return false;
            }
        }
    
        if (ki.size() == 0)
        {
            CONTROLIT_WARN << "Ki not specified.  Using a zero vector of size " << dimension << ".";
            ki.setZero(dimension);
        }
        else
        {
            if (ki.size() != dimension)
            {
                if (initDefault)
                    ki.setZero(dimension);
                else
                {
                    CONTROLIT_ERROR << "Size of ki incorrect.  Expected " << dimension << ", got " << ki.size();
                    return false;
                }
            }
        }
    
        if (kd.size() != dimension)
        {
            if (initDefault)
                kd.setZero(dimension);
            else
            {
                CONTROLIT_ERROR << "Size of kd incorrect.  Expected " << dimension << ", got " << kd.size();
                return false;
            }
        }
    
        e.resize(dimension);
        e_dot.resize(dimension);
    
        if (maxVelocity.size() != dimension)
        {
            CONTROLIT_WARN << "Size of maxVelocity incorrect.  Expected " << dimension
              << ", got " << maxVelocity.size()
              << ".  Setting maxVelocity to be a 1-vector of size " << dimension << ".";
    
            maxVelocity.setOnes(dimension);
        }
    
        if (maxAcceleration.size() != dimension)
        {
            CONTROLIT_WARN << "Size of maxAcceleration incorrect.  Expected " << dimension
              << ", got " << maxAcceleration.size()
              << ".  Setting maxAcceleration to be a zero-vector of size " << dimension << ".";
      
            maxAcceleration.setZero(dimension);
        }
    
        PDCommand.resize(dimension);
    
        integralTerm.setZero(dimension);
        lastIntegralTerm.setZero(dimension);
        untilityZero.setZero(dimension);
    
        int Nterms = static_cast<int>(integralPeriod / dt);
        for(int i = 0; i < Nterms; i++)
          lastNIntegralTerms.push_back(untilityZero);
    
        return true;
    }

    virtual bool computeCommand(Vector const& xd, Vector const& x,
        Vector const& xd_dot, Vector const& x_dot,
        Vector& u, controlit::ParameterReflection* pr)
    {
        PRINT_DEBUG_STATEMENT("Method called!\n"
           << " - xd = " << xd.transpose() << "\n"
              " - x = " << x.transpose() << "\n"
              " - xd_dot = " << xd_dot.transpose() << "\n"
              " - x_dot = " << x_dot.transpose())
    
        updateError(xd, x, xd_dot, x_dot, pr);
        bool result = computeCommandPolicy(u);
        paramCommand->set(u);
    
        PRINT_DEBUG_STATEMENT("Computed command: " << u.transpose() << ", result: " << result)
        return result;
    }

    virtual bool computeCommand(Vector const& err, Vector const& err_dot, Vector& u,
           controlit::ParameterReflection* pr)
    {
        // PRINT_DEBUG_STATEMENT("Method called!")
        updateError(err, err_dot, pr);
        bool result = computeCommandPolicy(u);
        paramCommand->set(u);
        return result;
    }
  
private:
    bool computeCommandPolicy(Vector& u)
    {
        u = (kp.array() * e.array() + kd.array() * e_dot.array()).matrix();
        return true;
    }

    void updateError(Vector const& xd, Vector const& x,
        Vector const& xd_dot, Vector const& x_dot,
        controlit::ParameterReflection* pr)
    {
        e = xd - x;
        e_norm = e.norm();
     
        e_dot = xd_dot - x_dot;
        e_dot_norm = e_dot.norm();
     
        paramError->set(e);
        paramErrorDot->set(e_dot);
        paramErrorNorm->set(e_norm);
        paramErrorDotNorm->set(e_dot_norm);
    }

    void updateError(Vector const& err, Vector const& err_dot,
        controlit::ParameterReflection* pr)
    {
        // PRINT_DEBUG_STATEMENT("Method Called!")
        e = err;
        e_norm = e.norm();
    
        e_dot = err_dot;
        e_dot_norm = e_dot.norm();
    
        // Publish the errors
        paramError->set(e);
        paramErrorDot->set(e_dot);
        paramErrorNorm->set(e_norm);
        paramErrorDotNorm->set(e_dot_norm);
    }

    Gain kp, kd;
    Vector e, e_dot;
    Vector PDCommand;
    double e_norm, e_dot_norm;
    Parameter maxVelocity, maxAcceleration;

    // Parameters for the integral term
    Gain ki;
    Parameter integralSaturation;
    int integralOn;
    size_t windUpCount;
    double integralPeriod;
    double dt;
  
    // Keeping track of integral term
    Vector integralTerm, lastIntegralTerm, untilityZero;
    std::list<Vector> lastNIntegralTerms;

    /*!
     * Persistent pointers to parameters.  These pointers are maintained to prevent
     * having to call lookupParameter(...) every time getCommand(...) is called.
     */
    controlit::Parameter * paramError;
    controlit::Parameter * paramErrorDot;
    controlit::Parameter * paramErrorNorm;
    controlit::Parameter * paramErrorDotNorm;
    controlit::Parameter * paramCommand;
    controlit::Parameter * paramIntegralTerm;
};


/****************************************************************************
 * ComponentWiseVel saturation policy
 ****************************************************************************/
template<>
struct PDControllerTraits<interface::SaturationPolicy::ComponentWiseVel>
{
    typedef Vector  Gain;
    typedef Vector  Parameter;
};

template<>
bool PDController<interface::SaturationPolicy::ComponentWiseVel>::resize(int dimension, bool initDefault)
{
    // PRINT_DEBUG_STATEMENT("Method Called!");

    if (kp.size() != dimension)
    {
        if (initDefault)
            kp.setZero(dimension);
        else
        {
            CONTROLIT_ERROR << "Size of kp incorrect.  Expected " << dimension << ", got " << kp.size();
            return false;
        }
    }

    if (ki.size() == 0)
    {
        CONTROLIT_WARN << "Ki not specified.  Using a zero vector of size " << dimension << ".";
        ki.setZero(dimension);
    }
    else
    {
        if (ki.size() != dimension)
        {
            if (initDefault)
                ki.setZero(dimension);
            else
            {
                CONTROLIT_ERROR << "Size of ki incorrect.  Expected " << dimension << ", got " << ki.size();
                return false;
            }
        }
    }

    if (kd.size() != dimension)
    {
        if (initDefault)
            kd.setZero(dimension);
        else
        {
            CONTROLIT_ERROR << "Size of kd incorrect.  Expected " << dimension << ", got " << kd.size();
            return false;
        }
    }
  
    e.resize(dimension);
    e_dot.resize(dimension);
  
    if (maxVelocity.size() != dimension)
    {
        CONTROLIT_WARN << "Size of maxVelocity incorrect.  Expected " << dimension
                       << ", got " << maxVelocity.size()
                       << ".  Setting maxVelocity to be a 1-vector of size " << dimension << ".";
    
        maxVelocity.setOnes(dimension);
    }
  
    if (maxAcceleration.size() != dimension)
    {
        CONTROLIT_WARN << "Size of maxAcceleration incorrect.  Expected " << dimension
                       << ", got " << maxAcceleration.size()
                       << ".  Setting maxAcceleration to be a zero-vector of size " << dimension << ".";
    
        maxAcceleration.setZero(dimension);
    }
  
    PDCommand.resize(dimension);
    integralSaturation.resize(dimension);
  
    integralTerm.setZero(dimension);
    lastIntegralTerm.setZero(dimension);
    untilityZero.setZero(dimension);
  
    int Nterms = static_cast<int>(integralPeriod / dt);
    for(int i = 0; i < Nterms; i++)
        lastNIntegralTerms.push_back(untilityZero);
  
    return true;
}

template<>
bool PDController<interface::SaturationPolicy::ComponentWiseVel>::computeCommandPolicy(Vector& u)
{
    PRINT_DEBUG_STATEMENT_NOPOLICY("(ComponentWiseVel): Method Called!\n"
        << " - integralOn = " << integralOn << "\n"
        << " - kp = " << kp.transpose() << "\n"
        << " - ki = " << ki.transpose() << "\n"
        << " - kd = " << kd.transpose() << "\n"
        << " - e = " << e.transpose() << "\n"
        << " - eDot = " << e_dot.transpose())

    assert(kp.size() == e.size());
    u = (kp.array() * e.array()).matrix();
  
    PRINT_DEBUG_STATEMENT_NOPOLICY("(ComponentWiseVel): Before velocity saturation:\n"
        << " - u = " << u.transpose() << "\n"
        << " - maxVelocity = " << maxVelocity.transpose())
  
    // PRINT_DEBUG_STATEMENT("u prior to saturation: = " << u.transpose());
  
    for (int row = 0; row < u.rows(); ++row)
    {
        if (std::abs(maxVelocity[row]) > 1e-6 && std::abs(kd[row]) > 1e-6 )   // beware of div by zero
        {
            double sat = std::fabs( u[row] / (maxVelocity[row] * kd[row]) );
            if (sat > 1.0) u[row] /= sat;
        }
    }
  
    PRINT_DEBUG_STATEMENT_NOPOLICY("(ComponentWiseVel): After velocity saturation:\n"
        << " - u = " << u.transpose())
  
    // PRINT_DEBUG_STATEMENT("u after saturation: = " << u.transpose());
  
    assert(kd.size() == e_dot.size());
  
    u += (kd.array() * e_dot.array()).matrix();
  
    PRINT_DEBUG_STATEMENT_NOPOLICY("(ComponentWiseVel): After adding damping:\n"
        << " - u = " << u.transpose())
  
    if (!controlit::addons::eigen::checkMagnitude(u))
    {
        CONTROLIT_ERROR << "(ComponentWiseVel): Command contains invalid values after adding damping:\n"
            << "  - u = " << u.transpose() << "\n"
            << " - integralOn = " << integralOn << "\n"
            << " - kp = " << kp.transpose() << "\n"
            << " - ki = " << ki.transpose() << "\n"
            << " - kd = " << kd.transpose() << "\n"
            << " - e = " << e.transpose() << "\n"
            << " - eDot = " << e_dot.transpose();
        return false;
    }
  
    if(integralOn)
    {
        assert(ki.size() == e.size());
    
        integralTerm = dt * (ki.array() * e.array()).matrix();
        lastNIntegralTerms.push_back(integralTerm);
    
        integralTerm = lastIntegralTerm + lastNIntegralTerms.back();
        integralTerm -= lastNIntegralTerms.front(); //zeros until wound up!
    
        windUpCount += 1;
        if(windUpCount > lastNIntegralTerms.size())
            windUpCount = lastNIntegralTerms.size();
    
        Vector modifiedIntegralTerm = integralTerm;//  * e.norm(); // A hack to make the Ki proportional to the error
    
        // for(int i = 0; i < modifiedIntegralTerm.size(); i++)
        //   if(fabs(modifiedIntegralTerm(i)) > integralSaturation(i))
        //     modifiedIntegralTerm(i) = integralSaturation(i) * modifiedIntegralTerm(i) / fabs(modifiedIntegralTerm(i));
    
        paramIntegralTerm->set(modifiedIntegralTerm);
        u += integralTerm;
    
        lastNIntegralTerms.pop_front();
        lastIntegralTerm = integralTerm;
    }
    // else
    // {
    //   lastNIntegralTerms.push_back(untilityZero);
    //   if(windUpCount > 0)
    //   {
    //     integralTerm  = lastIntegralTerm - lastNIntegralTerms.front();
    //     windUpCount -= 1;
    //   }
    // }
  
    // PRINT_DEBUG_STATEMENT("Done computing command.\n"
    //      " - integralTerm = " << integralTerm.transpose() << "\n"
    //      " - u = " << u.transpose());
  
    return true;
}

/*****************************************************************************
 * MaxComponentVel saturation policy
 *****************************************************************************/
// template<>
// struct PDControllerTraits<interface::SaturationPolicy::MaxComponentVel>
// {
//   typedef Vector  Gain;
//   typedef Vector  Parameter;
// };

// template<>
// bool PDController<interface::SaturationPolicy::MaxComponentVel>::resize(int dimension)
// {
//   e.resize(dimension);
//   e_dot.resize(dimension);
//   //assert(kp.size()==dimension);
//   return true;
// }

// template<>
// bool PDController<interface::SaturationPolicy::MaxComponentVel>::computeCommandPolicy(Vector& u)
// {
//   u = (kp.array() * e.array()).matrix();

//   double saturation = 0.0;
//   for (int row = 0; row < u.rows(); ++row)
//   {
//     if ( fabs(maxVelocity[row]) > 1e-6 && fabs(kd[row]) > 1e-6 )   // beware of div by zero
//     {
//       double sat = fabs( u[row] / (maxVelocity[row] * kd[row]) );
//       if (sat > saturation) saturation = sat;
//     }
//   }

//   if (saturation > 1.0) u /= saturation;

//   u += (kd.array() * e_dot.array()).matrix();

//   return true;
// }


/****************************************************************************
 * NormVel saturation policy
 ****************************************************************************/
template<>
struct PDControllerTraits<interface::SaturationPolicy::NormVel>
{
  typedef double  Gain;
  typedef double  Parameter;
};

template<>
bool PDController<interface::SaturationPolicy::NormVel>::resize(int dimension, bool initDefault)
{
  e.resize(dimension);
  e_dot.resize(dimension);

  integralTerm.setZero(dimension);
  lastIntegralTerm.setZero(dimension);
  untilityZero.setZero(dimension);

  int Nterms = int(integralPeriod / dt);
  for(int i = 0; i < Nterms; i++)
    lastNIntegralTerms.push_back(untilityZero);
  return true;
}

template<>
bool PDController<interface::SaturationPolicy::NormVel>::computeCommandPolicy(Vector& u)
{
  u = kp * e;

  if ( fabs(maxVelocity) > 1e-6 && fabs(kd) > 1e-6 )   // beware of div by zero
  {
    double sat = u.norm() / (maxVelocity * kd);
    if (sat > 1.0) u /= sat;
  }

  u += kd * e_dot;

  if(integralOn)
  {
    integralTerm = dt * ki * e;
    lastNIntegralTerms.push_back(integralTerm);

    integralTerm = lastIntegralTerm + lastNIntegralTerms.back();
    integralTerm -= lastNIntegralTerms.front(); //zeros until wound up!

    windUpCount += 1;
    if(windUpCount > lastNIntegralTerms.size())
      windUpCount = lastNIntegralTerms.size();

    // integralTerm = integralPeriod * (ki.array() * cumError.array()).matrix();
    //Safety/saturation
    Vector modifiedIntegralTerm = integralTerm;// * e.norm(); // A hack to make the Ki proportional to the error

    for(int i = 0; i < modifiedIntegralTerm.size(); i++)
      if(fabs(modifiedIntegralTerm(i)) > integralSaturation)
        modifiedIntegralTerm(i) = integralSaturation * modifiedIntegralTerm(i) / fabs(modifiedIntegralTerm(i));

    paramIntegralTerm->set(modifiedIntegralTerm);
    u += integralTerm;

    //clean up and book keeping
    lastNIntegralTerms.pop_front();
    lastIntegralTerm = integralTerm;
  }
  // else
  // {
  //   lastNIntegralTerms.push_back(untilityZero);
  //   if(windUpCount > 0)
  //   {
  //     integralTerm  = lastIntegralTerm - lastNIntegralTerms.front();
  //     windUpCount -= 1;
  //   }
  // }

  // PRINT_DEBUG_STATEMENT("Done computing command.\n"
  //      " - integralTerm = " << integralTerm.transpose() << "\n"
  //      " - u = " << u.transpose());
  return true;
}

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

/*****************************************************************************
 * ComponentWiseAcc saturation policy
 *****************************************************************************/
// template<>
// struct PDControllerTraits<interface::SaturationPolicy::ComponentWiseAcc>
// {
//   typedef Vector  Gain;
//   typedef Vector  Parameter;
// };

// template<>
// bool PDController<interface::SaturationPolicy::ComponentWiseAcc>::resize(int dimension)
// {
//   e.resize(dimension);
//   e_dot.resize(dimension);
//   //assert(kp.size()==dimension);
//   return true;
// }

// template<>
// bool PDController<interface::SaturationPolicy::ComponentWiseAcc>::computeCommandPolicy(Vector& u)
// {
//   u = (kp.array() * e.array()).matrix();
//   double commandNorm, satVel, satAcc;

//   for (int row = 0; row < u.rows(); ++row)
//   {
//     commandNorm=fabs(u[row]);
//     if(commandNorm>1e-6) //beware div by zero
//     {
//       satVel = fabs(kd[row]*maxVelocity[row]/commandNorm);
//       satAcc = fabs(maxAcceleration[row]-kd[row]*e_dot[row])/commandNorm;
//       if(satVel<satAcc&&satVel<1.0) u[row]*=satVel;
//       if(satAcc<satVel&&satAcc<1.0) u[row]*=satAcc;
//     }
//   }

//   u += (kd.array() * e_dot.array()).matrix();

//   return true;
// }


/*****************************************************************************
 * MaxComponentAcc saturation policy
 *****************************************************************************/
// template<>
// struct PDControllerTraits<interface::SaturationPolicy::MaxComponentAcc>
// {
//   typedef Vector  Gain;
//   typedef Vector  Parameter;
// };

// template<>
// bool PDController<interface::SaturationPolicy::MaxComponentAcc>::resize(int dimension)
// {
//   e.resize(dimension);
//   e_dot.resize(dimension);
//   //assert(kp.size()==dimension);
//   return true;
// }

// template<>
// bool PDController<interface::SaturationPolicy::MaxComponentAcc>::computeCommandPolicy(Vector& u)
// {
//   u = (kp.array() * e.array()).matrix();
//   double commandNorm, satVel, satAcc;
//   double saturation = 0.0;

//   for (int row = 0; row < u.rows(); ++row)
//   {
//     commandNorm=fabs(u[row]);
//     if(commandNorm>1e-6) //beware div by zero
//     {
//       satVel = fabs(kd[row]*maxVelocity[row]/commandNorm);
//       satAcc = fabs(maxAcceleration[row]-kd[row]*e_dot[row])/commandNorm;
//       if(satVel<satAcc&&satVel<1.0) saturation=satVel;
//       if(satAcc<satVel&&satAcc<1.0) saturation=satAcc;
//     }
//   }

//   if (saturation > 1.0) u *= saturation;

//   u += (kd.array() * e_dot.array()).matrix();

//   return true;
// }


// /*****************************************************************************
//  * NormAcc saturation policy
//  *****************************************************************************/
// template<>
// struct PDControllerTraits<interface::SaturationPolicy::NormAcc>
// {
//   typedef double  Gain;
//   typedef double  Parameter;
// };

// template<>
// bool PDController<interface::SaturationPolicy::NormAcc>::resize(int dimension)
// {
//   e.resize(dimension);
//   e_dot.resize(dimension);
//   return true;
// }

// template<>
// bool PDController<interface::SaturationPolicy::NormAcc>::computeCommandPolicy(Vector& u)
// {
//   u = kp * e;
//   double commandNorm = u.norm();

//   if ( fabs(commandNorm) > 1e-6 )   // beware of div by zero
//   {
//     double satVel = fabs(maxVelocity * kd)/commandNorm;
//     double satAcc = fabs(maxAcceleration-kd*e_dot_norm)/commandNorm;
//     if(satVel<satAcc&&satVel<1.0) u*=satVel;
//     if(satAcc<satVel&&satAcc<1.0) u*=satAcc;
//   }

//   u += kd * e_dot;

//   return true;
// }

} // namespace impl


/*****************************************************************************
 * PDController Factory
 *****************************************************************************/
namespace controlit {
namespace task_library {

PDController* PDControllerFactory::create(SaturationPolicy::Options policy)
{
  switch (policy)
  {
    case SaturationPolicy::Off: return new impl::PDController<SaturationPolicy::Off>();
    case SaturationPolicy::ComponentWiseVel: return new impl::PDController<SaturationPolicy::ComponentWiseVel>();
    // case SaturationPolicy::MaxComponentVel: return new impl::PDController<SaturationPolicy::MaxComponentVel>();
    case SaturationPolicy::NormVel: return new impl::PDController<SaturationPolicy::NormVel>();
    // case SaturationPolicy::ComponentWiseAcc: return new impl::PDController<SaturationPolicy::ComponentWiseAcc>();
    // case SaturationPolicy::MaxComponentAcc: return new impl::PDController<SaturationPolicy::MaxComponentAcc>();
    // case SaturationPolicy::NormAcc: return new impl::PDController<SaturationPolicy::NormAcc>();
    default: return NULL;
  }
}

} // namespace task_library
} // namespace controlit
