/*
 * DivisionByZeroException.h
 *
 *  Created on: Mar 21, 2017
 *      Author: ramiz
 */

#ifndef DIVISIONBYZEROEXCEPTION_H_
#define DIVISIONBYZEROEXCEPTION_H_
#include <exception>

class DivisionByZeroException : public std::exception {
  const char * what() const throw() {
      return "CalculateJacobian() - Error - Division by zero";
  }
};



#endif /* DIVISIONBYZEROEXCEPTION_H_ */
