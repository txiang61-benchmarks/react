/*
 * This file is part of React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://danielchappuis.ch>
 * React is re-licensed with permission from ReactPhysics3D author.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package com.flowpowered.react.constraint;

import units.qual.Dimensionless;
import java.util.List;

import gnu.trove.map.TObjectIntMap;

import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.engine.Island;
import com.flowpowered.react.math.Quaternion;
import com.flowpowered.react.math.Vector3;

/**
 * This class represents the constraint solver that is used to solve constraints between the rigid bodies. The constraint solver is based on the "Sequential Impulse" technique described by Erin Catto
 * in his GDC slides (http://code.google.com/p/box2d/downloads/list).
 * <p/>
 * A constraint between two bodies is represented by a function C(x) which is equal to zero when the constraint is satisfied. The condition C(x)=0 describes a valid position and the condition
 * dC(x)/dt=0 describes a valid velocity. We have dC(x)/dt = Jv + b = 0 where J is the Jacobian matrix of the constraint, v is a vector that contains the velocity of both bodies and b is the
 * constraint bias. We are looking for a force F_c that will act on the bodies to keep the constraint satisfied. Note that from the virtual work principle, we have F_c = J^t * lambda where J^t is the
 * transpose of the Jacobian matrix and lambda is a Lagrange multiplier. Therefore, finding the force F_c is equivalent to finding the Lagrange multiplier lambda.
 * <p/>
 * An impulse P = F * dt where F is a force and dt is the timestep. We can apply impulses a body to change its velocity. The idea of the Sequential Impulse technique is to apply impulses to bodies of
 * each constraints in order to keep the constraint satisfied.
 * <p/>
 * --- Step 1 ---
 * <p/>
 * First, we integrate the applied force F_a acting of each rigid body (like gravity, ...) and we obtain some new velocities v2' that tends to violate the constraints.
 * <p/>
 * v2' = v1 + dt * M^-1 * F_a
 * <p/>
 * where M is a matrix that contains mass and inertia tensor information.
 * <p/>
 * --- Step 2 ---
 * <p/>
 * During the second step, we iterate over all the constraints for a certain number of iterations and for each constraint we compute the impulse to apply to the bodies needed so that the new velocity
 * of the bodies satisfies Jv + b = 0. From the Newton law, we know that M * deltaV = P_c where M is the mass of the body, deltaV is the difference of velocity and P_c is the constraint impulse to
 * apply to the body. Therefore, we have v2 = v2' + M^-1 * P_c. For each constraint, we can compute the Lagrange multiplier lambda using : lambda = -m_c (Jv2' + b) where m_c = 1 / (J * M^-1 * J^t).
 * Now that we have the Lagrange multiplier lambda, we can compute the impulse P_c = J^t * lambda * dt to apply to the bodies to satisfy the constraint.
 * <p/>
 * --- Step 3 ---
 * <p/>
 * In the third step, we integrate the new position x2 of the bodies using the new velocities v2 computed in the second step with : x2 = x1 + dt * v2.
 * <p/>
 * Note that in the following code (as it is also explained in the slides from Erin Catto), the value lambda is not only the lagrange multiplier but is the multiplication of the Lagrange multiplier
 * with the timestep dt. Therefore, in the following code, when we use lambda, we mean (lambda * dt).
 * <p/>
 * We are using the accumulated impulse technique that is also described in the slides from Erin Catto.
 * <p/>
 * We are also using warm starting. The idea is to warm start the solver at the beginning of each step by applying the last impulses for the constraints that we already existing at the previous step.
 * This allows the iterative solver to converge faster towards the solution.
 * <p/>
 * For contact constraints, we are also using split impulses so that the position correction that uses Baumgarte stabilization does not change the momentum of the bodies.
 * <p/>
 * There are two ways to apply the friction constraints. Either the friction constraints are applied at each contact point or they are applied only at the center of the contact manifold between two
 * bodies. If we solve the friction constraints at each contact point, we need two constraints (two tangential friction directions) and if we solve the friction constraints at the center of the
 * contact manifold, we need two constraints for tangential friction but also another twist friction constraint to prevent spin of the body around the contact manifold center.
 */
@Dimensionless
public class ConstraintSolver {
    private @Dimensionless Vector3 @Dimensionless [] mLinearVelocities;
    private @Dimensionless Vector3 @Dimensionless [] mAngularVelocities;
    private final @Dimensionless List<@Dimensionless Vector3> mPositions;
    private final @Dimensionless List<@Dimensionless Quaternion> mOrientations;
    private final @Dimensionless TObjectIntMap<@Dimensionless RigidBody> mMapBodyToConstrainedVelocityIndex;
    private @Dimensionless float mTimeStep;
    private @Dimensionless boolean mIsWarmStartingActive;
    private final @Dimensionless ConstraintSolverData mConstraintSolverData;

    /**
     * Constructs a new constraint solver from positions, orientations and body to constrained velocity index map.
     *
     * @param positions The positions
     * @param orientations The orientations
     * @param mapBodyToConstrainedVelocityIndex The map from body to constrained velocity
     */
    public ConstraintSolver(@Dimensionless List<@Dimensionless Vector3> positions, @Dimensionless List<@Dimensionless Quaternion> orientations, @Dimensionless TObjectIntMap<@Dimensionless RigidBody> mapBodyToConstrainedVelocityIndex) {
        mLinearVelocities = null;
        mAngularVelocities = null;
        mPositions = positions;
        mOrientations = orientations;
        mMapBodyToConstrainedVelocityIndex = mapBodyToConstrainedVelocityIndex;
        mIsWarmStartingActive = true;
        mConstraintSolverData = new @Dimensionless ConstraintSolverData(positions, orientations, mapBodyToConstrainedVelocityIndex);
    }

    /**
     * Initializes the constraint solver for a given island.
     *
     * @param dt The time delta
     * @param island The island
     */
    public void initializeForIsland(@Dimensionless ConstraintSolver this, @Dimensionless float dt, @Dimensionless Island island) {
        if (mLinearVelocities == null) {
            throw new @Dimensionless IllegalStateException("Linear velocities cannot be null");
        }
        if (mAngularVelocities == null) {
            throw new @Dimensionless IllegalStateException("Angular velocities cannot be null");
        }
        if (island == null) {
            throw new @Dimensionless IllegalArgumentException("Island cannot be null");
        }
        if (island.getNbBodies() <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("The number of bodies in the island must be greater than zero");
        }
        if (island.getNbJoints() <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("The number of joints in the island must be greater than zero");
        }
        mTimeStep = dt;
        mConstraintSolverData.setTimeStep(mTimeStep);
        mConstraintSolverData.setWarmStartingActive(mIsWarmStartingActive);
        final @Dimensionless Joint @Dimensionless [] joints = island.getJoints();
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < island.getNbJoints(); i++) {
            joints[i].initBeforeSolve(mConstraintSolverData);
            if (mIsWarmStartingActive) {
                joints[i].warmstart(mConstraintSolverData);
            }
        }
    }

    /**
     * Solves the velocity constraints.
     *
     * @param island The island to solve
     */
    public void solveVelocityConstraints(@Dimensionless ConstraintSolver this, @Dimensionless Island island) {
        if (island == null) {
            throw new @Dimensionless IllegalArgumentException("Island cannot be null");
        }
        if (island.getNbJoints() <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("The number of joints in the island must be greater than zero");
        }
        final @Dimensionless Joint @Dimensionless [] joints = island.getJoints();
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < island.getNbJoints(); i++) {
            joints[i].solveVelocityConstraint(mConstraintSolverData);
        }
    }

    /**
     * Solves the position constraints.
     *
     * @param island The island to solve
     */
    public void solvePositionConstraints(@Dimensionless ConstraintSolver this, @Dimensionless Island island) {
        if (island == null) {
            throw new @Dimensionless IllegalArgumentException("Island cannot be null");
        }
        // TODO: this doesn't make sense
        //if (island.getNbJoints() <= 0) {
        //    throw new IllegalArgumentException("The number of joints in the island must be greater than zero");
        //}
        final @Dimensionless Joint @Dimensionless [] joints = island.getJoints();
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < island.getNbJoints(); i++) {
            joints[i].solvePositionConstraint(mConstraintSolverData);
        }
    }

    /**
     * Sets the constrained velocities arrays.
     *
     * @param constrainedLinearVelocities The constrained linear velocities
     * @param constrainedAngularVelocities The constrained angular velocities
     */
    public void setConstrainedVelocitiesArrays(@Dimensionless ConstraintSolver this, @Dimensionless Vector3 @Dimensionless [] constrainedLinearVelocities, @Dimensionless Vector3 @Dimensionless [] constrainedAngularVelocities) {
        if (constrainedLinearVelocities == null) {
            throw new @Dimensionless IllegalArgumentException("The constrained linear velocities cannot be null");
        }
        if (constrainedAngularVelocities == null) {
            throw new @Dimensionless IllegalArgumentException("The constrained angular velocities cannot be null");
        }
        mLinearVelocities = constrainedLinearVelocities;
        mAngularVelocities = constrainedAngularVelocities;
        mConstraintSolverData.setLinearVelocities(mLinearVelocities);
        mConstraintSolverData.setAngularVelocities(mAngularVelocities);
    }

    /**
     * This structure contains data from the constraint solver that is used to solve each joint constraint.
     */
    @Dimensionless
    public static class ConstraintSolverData {
        private @Dimensionless float timeStep;
        private @Dimensionless Vector3 @Dimensionless [] linearVelocities;
        private @Dimensionless Vector3 @Dimensionless [] angularVelocities;
        private final @Dimensionless List<@Dimensionless Vector3> positions;
        private final @Dimensionless List<@Dimensionless Quaternion> orientations;
        private final @Dimensionless TObjectIntMap<@Dimensionless RigidBody> mapBodyToConstrainedVelocityIndex;
        private @Dimensionless boolean isWarmStartingActive;

        /**
         * Constructs a new constraint solver data from the position, the orientations and the map from body to constrained velocity index.
         *
         * @param refPositions The positions
         * @param refOrientations The orientations
         * @param refMapBodyToConstrainedVelocityIndex The map from body to constrained velocity index
         */
        public ConstraintSolverData(ConstraintSolver.@Dimensionless ConstraintSolverData this, @Dimensionless List<@Dimensionless Vector3> refPositions, @Dimensionless List<@Dimensionless Quaternion> refOrientations,
                                    @Dimensionless
                                    TObjectIntMap<@Dimensionless RigidBody> refMapBodyToConstrainedVelocityIndex) {
            this.linearVelocities = null;
            this.angularVelocities = null;
            this.positions = refPositions;
            this.orientations = refOrientations;
            this.mapBodyToConstrainedVelocityIndex = refMapBodyToConstrainedVelocityIndex;
        }

        /**
         * Returns the time step.
         *
         * @return The time step
         */
        public @Dimensionless float getTimeStep(ConstraintSolver.@Dimensionless ConstraintSolverData this) {
            return timeStep;
        }

        /**
         * Sets the time step.
         *
         * @param timeStep The time step
         */
        public void setTimeStep(ConstraintSolver.@Dimensionless ConstraintSolverData this, @Dimensionless float timeStep) {
            this.timeStep = timeStep;
        }

        /**
         * Returns the list of linear velocities.
         *
         * @return The linear velocities
         */
        public @Dimensionless Vector3 @Dimensionless [] getLinearVelocities(ConstraintSolver.@Dimensionless ConstraintSolverData this) {
            return linearVelocities;
        }

        /**
         * Sets the linear velocities.
         *
         * @param linearVelocities The linear velocities
         */
        public void setLinearVelocities(ConstraintSolver.@Dimensionless ConstraintSolverData this, @Dimensionless Vector3 @Dimensionless [] linearVelocities) {
            this.linearVelocities = linearVelocities;
        }

        /**
         * Returns the list of angular velocities.
         *
         * @return The angular velocities
         */
        public @Dimensionless Vector3 @Dimensionless [] getAngularVelocities(ConstraintSolver.@Dimensionless ConstraintSolverData this) {
            return angularVelocities;
        }

        /**
         * Sets the angular velocities.
         *
         * @param angularVelocities The angular velocities
         */
        public void setAngularVelocities(ConstraintSolver.@Dimensionless ConstraintSolverData this, @Dimensionless Vector3 @Dimensionless [] angularVelocities) {
            this.angularVelocities = angularVelocities;
        }

        /**
         * Returns the list of positions.
         *
         * @return The positions
         */
        public @Dimensionless List<@Dimensionless Vector3> getPositions(ConstraintSolver.@Dimensionless ConstraintSolverData this) {
            return positions;
        }

        /**
         * Returns the list of orientations.
         *
         * @return The orientations
         */
        public @Dimensionless List<@Dimensionless Quaternion> getOrientations(ConstraintSolver.@Dimensionless ConstraintSolverData this) {
            return orientations;
        }

        /**
         * Returns the map from body to constrained velocity index.
         *
         * @return A map with a really long name that I'm tired of writing down
         */
        public @Dimensionless TObjectIntMap<@Dimensionless RigidBody> getMapBodyToConstrainedVelocityIndex(ConstraintSolver.@Dimensionless ConstraintSolverData this) {
            return mapBodyToConstrainedVelocityIndex;
        }

        /**
         * Returns true if warm starting is active.
         *
         * @return Whether or not warm starting is active
         */
        public @Dimensionless boolean isWarmStartingActive(ConstraintSolver.@Dimensionless ConstraintSolverData this) {
            return isWarmStartingActive;
        }

        /**
         * Sets whether or not warm starting is active.
         *
         * @param isWarmStartingActive Whether or not to warm start
         */
        public void setWarmStartingActive(ConstraintSolver.@Dimensionless ConstraintSolverData this, @Dimensionless boolean isWarmStartingActive) {
            this.isWarmStartingActive = isWarmStartingActive;
        }
    }
}
