/*
 *  Copyright (c) 2012-2013, MYOROBOTICS consortium
 *  Author: Steffen Wittmeier
 *  All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification is governed by the MYOROBOTICS Non-Commercial Software
 *  License Agreement. See LICENSE file distributed with this work for
 *  additional information.
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under this license is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either expressed or
 *  implied. See the License for the specific language governing permissions
 *  and limitations under the License.
 *
 */

#include <rl/math/Matrix.h>
#include <rl/math/Vector.h>
//#include <QDomElement>

/**
 * \brief Actuator interface.
 *
 * Interface of an actuator (DC motor + gear + spindle) as described in \cite Wittmeier2013b.
 *
 * \f{align}{
	\frac{d}{dt}
   \begin{pmatrix}
			i \\
			\omega{}_{G}
	\end{pmatrix}
   & =
   \begin{pmatrix}
   		-\frac{R}{L} & -\frac{K_{e}\Phi_fN}{L} \\[0.5em]
   		\frac{K_{T}\Phi_f}{NJ_{\textrm{tot}}} & 0
   \end{pmatrix}
   \begin{pmatrix}
			i \\
			\omega{}_{G}
	\end{pmatrix}
   & +
   \begin{pmatrix}
       \frac{1}{L} & 0 \\[0.5em]
       0 & \frac{-r_S}{\rho{}N^2J_{\textrm{tot}}}
	\end{pmatrix}
   \begin{pmatrix}
			u \\
			f
   \end{pmatrix}\label{eq:motorModel}\\[0.5em]
\nonumber   \textrm{with}\hspace{1em} J_{\textrm{tot}} & = J_M + J_{G} \\[0.5em]
\nonumber   \textrm{and}\hspace{1em}
\rho{} & =
  \begin{cases}
   \eta & \text{if } \omega_G\cdot i \geq 0 \\
   1/\eta       & \text{if } \omega_G \cdot i < 0
  \end{cases}
\f}
 * \par XML-Definition of an Actuator
 *
 * Actuators can be instantiated for a muscle by adding the following XML
 * definition to a muscle-tag.
 *
 * \include de.caliper.sim/actuator.xml
 */
class IActuator
{
public:
	/**
	 * \brief Integrator used for the numerical integration of the actuator model.
	 */
	enum Integrator
	{
		ForwardEuler, RungeKutta4
	};

	/**
	 * \brief Destructor.
	 */
	virtual ~IActuator()
	{

	}

	/**
	 * \brief Initializes the actuator model.
	 *
	 * \param element QDomElement representing the root of the actuator definition
	 * in the muscle XML file.
	 */
//	virtual void init(const QDomElement& element) = 0;

	/**
	 * \brief Returns the anchor resistance \f$ R \f$.
	 *
	 * \return Anchor resistance in [ohm].
	 */
	virtual rl::math::Real getAnchorResistance() const = 0;

	/**
	 * \brief Sets the anchor resistance \f$ R \f$.
	 *
	 * \param anchorResistance Anchor resistance in [ohm].
	 */
	virtual void setAnchorResistance(rl::math::Real anchorResistance) = 0;

	/**
	 * \brief Returns the H-Bridge resistance.
	 *
	 * \return H-Bridge resistance in [ohm].
	 */
	virtual rl::math::Real getHbridgeResistance() const = 0;

	/**
	 * \brief Sets the H-Bridge resistance.
	 *
	 * \param hbridgeResistance H-Bridge resistance in [ohm].
	 */
	virtual void setHbridgeResistance(rl::math::Real hbridgeResistance) = 0;

	/**
	 * \brief Returns the anchor inductance \f$ L \f$.
	 *
	 * \return Anchor inductance in [H].
	 */
	virtual rl::math::Real getAnchorInductance() const = 0;

	/**
	 * \brief Sets the anchor inductance \f$ L \f$.
	 *
	 * \param anchorInductance Anchor inductance in [H].
	 */
	virtual void setAnchorInductance(rl::math::Real anchorInductance) = 0;

	/**
	 * \brief Returns the back-EMF constant \f$ K_e \phi_f \f$.
	 *
	 * \return Back-EMF constant in [Vs/rad].
	 */
	virtual rl::math::Real getBackEmfConstant() const = 0;

	/**
	 * \brief Sets the back-EMF constant \f$ K_e \phi_f \f$.
	 *
	 * \param backEmfConstant Back-EMF constant in [Vs/rad].
	 */
	virtual void setBackEmfConstant(rl::math::Real backEmfConstant) = 0;

	/**
	 * \brief Returns the torque constant \f$ K_t \phi_f \f$.
	 *
	 * \return Torque constant in [Nm/A].
	 */
	virtual rl::math::Real getTorqueConstant() const = 0;

	/**
	 * \brief Sets the torque constant \f$ K_t \phi_f \f$.
	 *
	 * \param torqueConstant Torque constant in [Nm/A].
	 */
	virtual void setTorqueConstant(rl::math::Real torqueConstant) = 0;

	/**
	 * \brief Returns the moment of inertia of the DC motor \f$ J_M \f$.
	 *
	 * \return Moment of inertia in [kgm^2].
	 */
	virtual rl::math::Real getMomentOfInertiaMotor() const = 0;

	/**
	 * \brief Sets moment of inertia of the DC motor \f$ J_M \f$.
	 *
	 * \param momentOfInertiaMotor Moment of inertia in [kgm^2].
	 */
	virtual void setMomentOfInertiaMotor(
			rl::math::Real momentOfInertiaMotor) =0;

	/**
	 * \brief Returns moment of inertia of the gear \f$ J_G \f$.
	 *
	 * \return Moment of inertia in [kgm^2].
	 */
	virtual rl::math::Real getMomentOfInertiaGearbox() const = 0;

	/**
	 * \brief Sets moment of inertia of the gear \f$ J_G \f$.
	 *
	 * \param momentOfInertiaGearbox Moment of inertia in [kgm^2].
	 */
	virtual void setMomentOfInertiaGearbox(
			rl::math::Real momentOfInertiaGearbox) = 0;

	/**
	 * \brief Returns the gear efficiency \f$ \eta \f$.
	 *
	 * \return Gear efficiency.
	 */
	virtual rl::math::Real getGearboxEfficiency() const = 0;

	/**
	 * \brief Sets the gear efficiency \f$ \eta \f$.
	 *
	 * \param gearboxEfficiency Gearbox efficiency.
	 */
	virtual void setGearboxEfficiency(rl::math::Real gearboxEfficiency) = 0;

	/**
	 * \brief Returns the gear ratio \f$ N \f$.
	 *
	 * \return Gear ratio.
	 */
	virtual rl::math::Real getGearboxRatio() const = 0;

	/**
	 * \brief Sets the gear ratio \f$ N \f$.
	 *
	 * \param gearboxRatio Gear ratio.
	 */
	virtual void setGearboxRatio(rl::math::Real gearboxRatio) = 0;

	/**
	 * \brief Returns the spindle radius \f$ r_S \f$.
	 *
	 * \return Spindle radius in [m].
	 */
	virtual rl::math::Real getSpindleRadius() const = 0;

	/**
	 * \brief Sets the spindle radius \f$ r_S \f$.
	 *
	 * \param spindleRadius Spindle radius in [m].
	 */
	virtual void setSpindleRadius(rl::math::Real spindleRadius) = 0;

	/**
	 * \brief Sets the input voltage \f$ u \f$.
	 *
	 * \param voltage Input voltage.
	 */
	virtual void setVoltage(rl::math::Real voltage) = 0;

	/**
	 * \brief Returns the set input voltage \f$ u \f$.
	 *
	 * \return Input voltage.
	 */
	virtual rl::math::Real getVoltage() const = 0;

	/**
	 * \brief Sets the load torque \f$ \tau_{LG}\f$.
	 *
	 * \param torque Load torque in [Nm].
	 */
	virtual void setLoadTorque(rl::math::Real torque) = 0;

	/**
	 * \brief Returns the angular velocity \f$ \omega_G \f$.
	 *
	 * \return Angular velocity of the gear output shaft in [rad/s].
	 */
	virtual rl::math::Real getAngularVelocity() const = 0;

	/**
	 * \brief Returns the linear velocity \f$ v_T \f$.
	 *
	 * \return Linear velocity of the tendon in [m/s].
	 */
	virtual rl::math::Real getLinearVelocity() const = 0;

	/**
	 * \brief Returns the electric current \f$ i \f$.
	 *
	 * \return Electric current in [A].
	 */
	virtual rl::math::Real getCurrent() const = 0;

	/**
	 * \brief Returns the angular position \f$ \theta_G \f$.
	 *
	 * \return Angular position in [deg].
	 */
	virtual rl::math::Real getPosition() const = 0;

	/**
	 * \brief Returns the initial angular position.
	 *
	 * \return Initial angular position in [deg].
	 */
	virtual rl::math::Real getInitialPosition() const = 0;

	/**
	 * \brief Steps the actuator model by \f$ \triangle{}t\f$.
	 *
	 * \param time Time in [s].
	 */
	virtual void step(rl::math::Real time) = 0;

	/**
	 * \brief Sets the time step of numerical integrator.
	 *
	 * \param timeStep Integrator step size in [s].
	 */
	virtual void setTimeStep(rl::math::Real timeStep) = 0;

	/**
	 * \brief Returns the time step of the integrator.
	 *
	 * \return Time step in [s].
	 */
	virtual rl::math::Real getTimeStep() = 0;

	/**
	 * \brief Sets the integrator type.
	 *
	 * \param integrator Integrator type.
	 */
	virtual void setIntegrator(Integrator integrator) = 0;

	/**
	 * \brief Returns the integrator type.
	 *
	 * \return Integrator type.
	 */
	virtual Integrator getIntegrator() = 0;

	/**
	 * \brief Prints the matrices of the state-space model to std::cout.
	 *
	 * \note Debug method.
	 */
	virtual void printModelMatrices() = 0;
};

