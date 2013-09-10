/*
 * SL_SolvePnP.h
 *
 *  Created on: 2011-1-18
 *      Author: Danping Zou
 */

#ifndef SL_SOLVEPNP_H_
#define SL_SOLVEPNP_H_
#include "extern/sba-1.6/sba.h"

/* iteratively estimate camera pose at single view 
 * ms : image points
 * Ms : scene points
 * R0,t0 : initial pose
 * R,t: final pose
 */
void solvePnPIter(
		int npts ,
		const double* ms ,
		const double* cov ,
		const double* Ms ,
		const double* R0 ,
		const double* t0 ,
		double* R ,
		double* t ,
		int maxIter ,
		double* err = 0);

void solvePnPIter(
		const double* K ,
		int npts ,
		const double* ms ,
		const double* cov ,
		const double* Ms ,
		const double* R0 ,
		const double* t0 ,
		double* R ,
		double* t ,
		int maxIter ,
		double* err = 0);

void solvePnPIter(
		const double* K ,
		const double* kc, 
		int npts ,
		const double* ms ,
		const double* cov ,
		const double* Ms ,
		const double* R0 ,
		const double* t0 ,
		double* R ,
		double* t ,
		int maxIter ,
		double* err = 0);

#endif /* SL_SOLVEPNP_H_ */
