//!
//! @file         Fp32s.hpp
//! @author       Geoffrey Hunter <gbmhunter@gmail.com> (www.mbedded.ninja)
//! @edited       n/a
//! @created      2013-07-22
//! @last-modified    2014-09-15
//! @brief        A slower, more powerful 32-bit fixed point library.
//! @details
//!   See README.rst in root dir for more info.

//===============================================================================================//
//====================================== HEADER GUARD ===========================================//
//===============================================================================================//
#include <stdint.h>

#ifndef __cplusplus
  #error Please build with C++ compiler
#endif

#ifndef FP32S_H
#define FP32S_H

//#include <stdint.h>

// Fixed-point configuration file
//#include "Config.hpp"

// Port-specific code
//#include "Port.hpp"

namespace Fp
{

  //! The template argument p in all of the following functions refers to the 
  //! fixed point precision (e.g. p = 8 gives 24.8 fixed point functions).
  //! Contains mathematical operator overloading. Doesn't have modulus (%) overloading
  class Fp32s {
    
    public:
    
    //! @brief    The fixed-point number is stored in this basic data type.
    int32_t rawVal;     
    
    //! @brief    This stores the number of fractional bits.
    uint8_t q;
    
    Fp32s()
    {
      //#if(fpConfig_PRINT_DEBUG_GENERAL == 1)
        //Port::DebugPrint("FP: New fixed-point object created.");
      //#endif
    }
    
    Fp32s(int32_t i, uint8_t qin)
    {
      rawVal = i << qin;
      q = qin;
    }


    Fp32s(int32_t i, uint8_t qin, bool do_shift)
    {
      if (do_shift) {
        rawVal = i << qin;
      }
      else {
        rawVal = i;
      }
      q = qin;
    }
    
    Fp32s(double dbl, uint8_t qin)
    {
      rawVal = (int32_t)(dbl * (1 << qin));
      q = qin;
    }

    Fp32s(float dbl, uint8_t qin)
    {
      rawVal = (int32_t)(dbl * (1 << qin));
      q = qin;
    }
    
    // Compound Arithmetic Operators
    
    //! @brief    Overload for '+=' operator.
    Fp32s& operator += (Fp32s r)
    { 
      // Optimised for when q is the same for both
      // operators (first if statement).
      if(q == r.q)
      {
        // First number smaller
        /*
        int64_t temp = (int64_t)rawVal + (int64_t)r.rawVal;
        if (temp>2147483648) {
          rawVal = (int32_t) temp>>1;
          q = q-1;
        }
        else {
          rawVal = (int32_t) temp;
        }
        */
        rawVal = rawVal + r.rawVal;
        // No need to change Q, both are the same
      }
      else if(q > r.q)
      {
        // Second number has smaller Q, so result is in that precision
        rawVal = (rawVal >> (q - r.q)) + r.rawVal; 
        // Change Q
        q = r.q;
      }
      else // q < r.q
      {
        // First number has smaller Q, so result is in that precision
        rawVal = rawVal + (r.rawVal >> (r.q - q)); 
        // No need to change Q
      }
      return *this;
    }
    
    //! @brief    Overload for '-=' operator.
    Fp32s& operator -= (Fp32s r)
    { 
      // Optimised for when q is the same for both
      // operators (first if statement).
      if(q == r.q)
      {
        // Q the same for both numbers
        rawVal = rawVal - r.rawVal;
        // No need to change Q, both are the same
      }
      else if(q > r.q)
      {
        // Second number has smaller Q, so result is in that precision
        rawVal = (rawVal >> (q - r.q)) - r.rawVal; 
        // Change Q
        q = r.q;
      }
      else // q < r.q
      {
        // First number has smaller Q, so result is in that precision
        rawVal = rawVal - (r.rawVal >> (r.q - q)); 
        // No need to change Q
      }
      return *this;
    }
    
    //! @brief    Overlaod for '*=' operator.
    //! @details  Uses intermediatary casting to int64_t to prevent overflows.
    Fp32s& operator *= (Fp32s r)
    {
      // Optimised for when q is the same for both
      // operators (first if statement).
      if(q == r.q)
      {
        // Q the same for both numbers, shift right by Q
        rawVal = (int32_t)(((int64_t)rawVal * (int64_t)r.rawVal) >> q);
        // No need to change Q, both are the same
      }
      else if(q > r.q)
      {
        // Second number has smaller Q, so result is in that precision
        rawVal = (int32_t)((((int64_t)rawVal >> (q - r.q)) * (int64_t)r.rawVal) >> r.q); 
        // Change Q
        q = r.q;
      }
      else // q < r.q
      {
        // First number has smaller Q, so result is in that precision
        rawVal = (int32_t)(((int64_t)rawVal * ((int64_t)r.rawVal >> (r.q - q))) >> q); 
        // No need to change Q
      }
      return *this;
    }
    
    //! @brief    Overlaod for '/=' operator.
    //! @details  Uses intermediatary casting to int64_t to prevent overflows.
    Fp32s& operator /= (Fp32s r)
    {
      // Optimised for when q is the same for both
      // operators (first if statement).
      if(q == r.q)
      {
        // Q the same for both numbers, shift right by Q

        rawVal = (int32_t)((((int64_t)rawVal << q) / (int64_t)r.rawVal));
        // No need to change Q, both are the same
      }
      else if(q > r.q)
      {
        // Second number has smaller Q, so result is in that precision
        rawVal = (int32_t)(((((int64_t)rawVal >> (q - r.q)) << r.q) / (int64_t)r.rawVal)); 
        // Change Q
        q = r.q;
      }
      else // q < r.q
      {
        // First number has smaller Q, so result is in that precision
        rawVal = (int32_t)(((int64_t)rawVal << q) / ((int64_t)r.rawVal >> (r.q - q))); 
        // No need to change Q
      }
      return *this;
    }
    
    //! @brief    Overlaod for '%=' operator.
    Fp32s& operator %= (Fp32s r)
    {
      // Optimised for when q is the same for both
      // operators (first if statement).
      if(q == r.q)
      {
        // Q the same for both numbers
        rawVal = rawVal % r.rawVal;
        // No need to change Q, both are the same
      }
      else if(q > r.q)
      {
        // Second number has smaller Q, so result is in that precision
        rawVal = (rawVal >> (q - r.q)) % r.rawVal; 
        // Change Q
        q = r.q;
      }
      else // q < r.q
      {
        // First number has smaller Q, so result is in that precision
        rawVal = rawVal % (r.rawVal >> (r.q - q)); 
        // No need to change Q
      }
      return *this;
    }
    
    // Simple Arithmetic Operators
    
    Fp32s operator ~ (void) const
    {
      Fp32s x = *this;
      x.rawVal = x.rawVal*-1;
      return x;
    }

    //! @brief    Overload for '+' operator.
    //! @details  Uses '+=' operator.
    Fp32s operator + (Fp32s r) const
    {
      Fp32s x = *this;
      x += r;
      return x;
    }
    
    //! @brief    Overload for '-' operator.
    //! @details  Uses '-=' operator.
    Fp32s operator - (Fp32s r) const
    {
      Fp32s x = *this;
      x -= r;
      return x;
    }
    
    //! @brief    Overload for '*' operator.
    //! @details  Uses '*=' operator.
    Fp32s operator * (Fp32s r) const
    {
      Fp32s x = *this;
      x *= r;
      return x;
    }
    
    //! @brief    Overload for '/' operator.
    //! @details  Uses '/=' operator.
    Fp32s operator / (Fp32s r) const
    {
      Fp32s x = *this;
      x /= r;
      return x;
    }
    
    //! @brief    Overload for '%' operator.
    //! @details  Uses '%=' operator.
    Fp32s operator % (Fp32s r) const
    {
      Fp32s x = *this;
      x %= r;
      return x;
    }
    
    // Binary Operator Overloads
    
    //! @brief    Overload for the '==' operator.
    bool operator == (Fp32s r) const
    {
      // Optimised for when q is the same for both
      // operators (first if statement).
      if(q == r.q)
      {
        // Q the same for both numbers
        return rawVal == r.rawVal;
      }
      else if(q > r.q)
      {
        // Second number has smaller Q, so result is in that precision
        return (rawVal >> (q - r.q)) == r.rawVal; 
      }
      else // q < r.q
      {
        // First number has smaller Q, so result is in that precision
        return rawVal == (r.rawVal >> (r.q - q)); 
      }
    }
    
    //! @brief    Overload for the '!=' operator.
    bool operator != (Fp32s r) const
    {
      // Optimised for when q is the same for both
      // operators (first if statement).
      if(q == r.q)
      {
        // Q the same for both numbers
        return rawVal != r.rawVal;
      }
      else if(q > r.q)
      {
        // Second number has smaller Q, so result is in that precision
        return (rawVal >> (q - r.q)) != r.rawVal; 
      }
      else // q < r.q
      {
        // First number has smaller Q, so result is in that precision
        return rawVal != (r.rawVal >> (r.q - q)); 
      }
    }
    
    //! @brief    Overload for the '<' operator.
    bool operator < (Fp32s r) const
    {
      // Optimised for when q is the same for both
      // operators (first if statement).
      if(q == r.q)
      {
        // Q the same for both numbers
        return rawVal < r.rawVal;
      }
      else if(q > r.q)
      {
        // Second number has smaller Q, so result is in that precision
        return (rawVal >> (q - r.q)) < r.rawVal; 
      }
      else // q < r.q
      {
        // First number has smaller Q, so result is in that precision
        return rawVal < (r.rawVal >> (r.q - q)); 
      }
    }

    Fp32s& operator >> (int r) {
      rawVal = rawVal >> r;
      q = q-r;
      return *this;
    }

    //! @brief    Overload for the '>' operator.
    bool operator > (Fp32s r) const
    {
      // Optimised for when q is the same for both
      // operators (first if statement).
      if(q == r.q)
      {
        // Q the same for both numbers
        return rawVal > r.rawVal;
      }
      else if(q > r.q)
      {
        // Second number has smaller Q, so result is in that precision
        return (rawVal >> (q - r.q)) > r.rawVal; 
      }
      else // q < r.q
      {
        // First number has smaller Q, so result is in that precision
        return rawVal > (r.rawVal >> (r.q - q)); 
      }
    }
    
    //! @brief    Overload for the '<=' operator.
    bool operator <= (Fp32s r) const
    {
      // Optimised for when q is the same for both
      // operators (first if statement).
      if(q == r.q)
      {
        // Q the same for both numbers
        return rawVal <= r.rawVal;
      }
      else if(q > r.q)
      {
        // Second number has smaller Q, so result is in that precision
        return (rawVal >> (q - r.q)) <= r.rawVal; 
      }
      else // q < r.q
      {
        // First number has smaller Q, so result is in that precision
        return rawVal <= (r.rawVal >> (r.q - q)); 
      }
    }
    
    //! @brief    Overload for the '>=' operator.
    bool operator >= (Fp32s r) const
    {
      // Optimised for when q is the same for both
      // operators (first if statement).
      if(q == r.q)
      {
        // Q the same for both numbers
        return rawVal >= r.rawVal;
      }
      else if(q > r.q)
      {
        // Second number has smaller Q, so result is in that precision
        return (rawVal >> (q - r.q)) >= r.rawVal; 
      }
      else // q < r.q
      {
        // First number has smaller Q, so result is in that precision
        return rawVal >= (r.rawVal >> (r.q - q)); 
      }
    }
    
    // Explicit Conversion Operator Overloads (casts)
    
    //! @brief    Conversion operator from fixed-point to int32_t.
    operator int32_t()
    {
      // Right-shift to get rid of all the decimal bits
      return (rawVal >> q);
    }
    
    //! @brief    Conversion operator from fixed-point to int64_t.
    operator int64_t()
    {
      // Right-shift to get rid of all the decimal bits
      return (int64_t)(rawVal >> q);
    }
    
    //! @brief    Conversion operator from fixed-point to float.
    //! @note   Similar to double conversion.
    operator float()
    { 
      return (float)rawVal / (float)(1 << q);
    }
    
    //! @brief    Conversion operator from fixed-point to double.
    //! @note   Similar to float conversion.
    operator double()
    { 
      return (double)rawVal / (double)(1 << q);
    }
    
  };

} // namespace Fp

#endif // #ifndef FP32S_H

// EOF

