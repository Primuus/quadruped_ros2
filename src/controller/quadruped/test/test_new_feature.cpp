#include <iostream>
#include <cstdlib>
#include "quadruped_leg/leg.hpp"

int main(int argc, char* argv[])
{
    std::cout << "Hello" << std::endl;

    QuadrupedLeg* _Legs = new QuadrupedLeg(0, 0.08, 0.213, 0.213, Vec3D(-0.1805,  0.047, 0));
    std::cout << "PosPEe2B" << std::endl;
    std::cout << _Legs->calcPEe2B(Vec3D(0.0, 0.67, -1.3))<< std::endl;
    std::cout << "PosPEe2H" << std::endl;
    std::cout << _Legs->calcPEe2H(Vec3D(0.0, 0.67, -1.3))<< std::endl;
    std::cout << "AngleHIP" << std::endl;
    std::cout << _Legs->calcQ(Vec3D(0.0, 0.67, -1.3), FrameType::HIP) << std::endl;
    std::cout << "AngleBODY" << std::endl;
    std::cout << _Legs->calcQ(Vec3D(0.0, 0.67, -1.3), FrameType::BODY) << std::endl;

    return 0;
}
