//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle_filter_test.cc
\brief   Google Test file for particle filter based
         mobile robot localization
\author  Melissa Cruz, Yuhong Kan, Maxx Wilson
*/
//========================================================================

#include <gtest/gtest.h>
#include "particle_filter.h"

class ParticleFilterTest: public ::testing::Test {
    protected:
  // You can remove any or all of the following functions if their bodies would
  // be empty.

  ParticleFilterTest() {
     // You can do set-up work for each test here.
  }

  ~ParticleFilterTest() override {
     // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override {
     // Code here will be called immediately after the constructor (right
     // before each test).
  }

  void TearDown() override {
     // Code here will be called immediately after each test (right
     // before the destructor).
  }

  // Class members declared here can be used by all tests in the test suite.
};

// TEST(ParticleFilterTest, TestResample) {
//     particle_filter::ParticleFilter filter;

//    std::vector<particle_filter::Particle> particles(100);

//    for(particle_filter::Particle particle: particles){
//       particle.loc = Eigen::Vector2f();
//       particle.angle = 0.0;
//       particle.weight = ;
//    }

//    filter.Resample();
// }

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}