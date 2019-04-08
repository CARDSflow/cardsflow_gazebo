#include "cardsflow_gazebo/eccerobotMuscle/physics/Actuator.h"
#include <iostream>
#include <fstream>
#include <vector>
//#include <rl-0.7.0/rl/util/Timer.h>

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: testMotor [integrator] [dt]" << std::endl;

        return 0;
    }

    Actuator motor;

    motor.setAnchorResistance(0.516);
    motor.setAnchorInductance(5.73e-5);
    motor.setBackEmfConstant(11.5e-3);
    motor.setTorqueConstant(11.5e-3);
    motor.setGearboxEfficiency(0.59);
    motor.setGearboxRatio(128);
    motor.setMomentOfInertiaGearbox(0.4e-7);
    motor.setMomentOfInertiaMotor(14.5e-7);

    std::cout << "Anchor resistance=" << motor.getAnchorResistance()
              << std::endl;
    std::cout << "Anchor inductance=" << motor.getAnchorInductance()
              << std::endl;
    std::cout << "Back-EMF=" << motor.getBackEmfConstant() << std::endl;
    std::cout << "Torque-Constant=" << motor.getTorqueConstant() << std::endl;
    std::cout << "Gearbox Efficiency=" << motor.getGearboxEfficiency()
              << std::endl;
    std::cout << "Gearbox Ratio=" << motor.getGearboxRatio() << std::endl;
    std::cout << "J-Gearbox=" << motor.getMomentOfInertiaGearbox() << std::endl;
    std::cout << "J-Motor=" << motor.getMomentOfInertiaMotor() << std::endl;

    motor.setVoltage(0.0);
    motor.setLoadTorque(0.0);

    //motor.setIntegrator((de::caliper::sim::ext::actuators::er::physics::Actuator::Integrator) atoi(argv[1]));
    motor.setIntegrator(
            Actuator::RungeKutta4);
    motor.setTimeStep(atof(argv[2]));

    motor.printModelMatrices();

    std::cout << "Integrator=" << motor.getIntegrator() << ", TimeStep="
              << motor.getTimeStep() << std::endl;

    std::ofstream file;

    file.open("motor.txt");

//    rl::util::Timer timer;

    std::vector< double > times;
    std::vector< double > current;
    std::vector< double > velocity;
    std::vector< double > rho;
    std::vector< double > x;

//    timer.start();

    for (double time = 0; time < 5.0; time += motor.getTimeStep())
    {
        if (time >= 1.0)
        {
            motor.setVoltage(3.0);
        }

        if (time > 3.0)
        {
            motor.setLoadTorque(10);
        }

        if (time > 4.0)
        {
            motor.setLoadTorque(0);
        }

        motor.step(motor.getTimeStep());

        times.push_back(time);

        current.push_back(motor.getCurrent());

        velocity.push_back(motor.getAngularVelocity());

        rho.push_back(motor.getRho());

        x.push_back(motor.getAngularVelocity() * motor.getCurrent());
    }

//    timer.stop();

//    std::cout << "Simulation took=" << timer.elapsed() << std::endl;

    std::cout << "vel=" << motor.getAngularVelocity() << std::endl;

    for (int i = 0; i < times.size(); i++)
    {
        file << times[i] << "\t" << current[i] << "\t" << velocity[i]
             << "\t" << rho[i] << "\t" << x[i] <<  std::endl;
    }

    file.close();

    return 0;
}
