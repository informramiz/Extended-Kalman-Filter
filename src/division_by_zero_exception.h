/*
 * DivisionByZeroException.h
 *
 *  Created on: Mar 21, 2017
 *      Author: ramiz
 */

#ifndef DIVISION_BY_ZERO_EXCEPTION_H_
#define DIVISION_BY_ZERO_EXCEPTION_H_
#include <exception>

class DivisionByZeroException : public std::exception {
  const char * what() const throw() {
      return "CalculateJacobian() - Error - Division by zero";
  }
};



#endif /* DIVISION_BY_ZERO_EXCEPTION_H_ */
