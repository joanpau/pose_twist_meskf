/*
 * test_matrix.c
 *
 *  Created on: 13/05/2012
 *      Author: usuari
 */

#include "analyticconditionalgaussian_posetwisterrorstate.h"

int main(int argc, char* argv[])
{
  const MatrixWrapper::Matrix m(15,15);
  std::cout << m << '\n';
  return 0;
}
