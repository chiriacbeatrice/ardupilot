/*
 * test_complexBC.cpp
 *
 *  Created on: May 27, 2019
 *      Author: beatrice
 */

#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Math/PolygonConvexBC.h"

// this line is necessary for linking, according to
// http://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(ComplexTest, Test0)
{

}

AP_GTEST_MAIN()
